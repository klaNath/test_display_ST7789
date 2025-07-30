"""
# MicropyGPS - a GPS NMEA sentence parser for Micropython/Python 3.X
# Copyright (c) 2017 Michael Calvin McCoy (calvin.mccoy@protonmail.com)
# The MIT License (MIT) - see LICENSE file
#
# Modfied by ekspla (https://github.com/ekspla), 2023.  Tested on CPython 3 and Micropython 1.19.1.
# 1. Fixed wrong formatted decimal degrees in latitudes and logitudes. DD format should have sign (+/-)
#    indicating N/S or E/W.  These properties in dd fromat are changed from tuples to float values.
# 2. Tuples (instead of lists) are used in most of the attributes for less memory consumption.
# 3. Change in speed attribute (knots only).  The unit conversions are trivial.  Use speed_string().
# 4. Undefined DOP is changed from float(0.0) to float('nan') because 0.0 has an opposite meaning.
# 5. Modified the code in update() for efficiency.  Unsupported gps sentences are not read any more;
#    use mutable supported_sentences (dict) to control.
# 6. Added ZDA parser from which centuries in years (int:19 or 20) can be read; the attribute updated.
# 7. In RMC/GGA/GLL/ZDA, timestamp and coordinate parsers are refactored.
# 8. f-strings are used in string formatting.  This requires relatively new (!= EOL) python versions.
# 9. Refactored pretty printing of latitude_string() and longitude_string().
# 10. Centuries in years are shown by s_dmy and s_mdy format in date_string().
# 11. Import of math module is not required.  Change in speed and efficiency was negligible without it.
# 12. Fixed assertion error of test_micropyGPS.test_logging() due to the teriminator (\r\n) on Windows.
"""

# TODO:
# Time Since First Fix
# Distance/Time to Target
# More Helper Functions
# Dynamically limit sentences types to parse

# Import utime or time for fix time handling
try:
    # Assume running on MicroPython
    import utime
    get_ticks = utime.ticks_ms # get_ticks() (in milli second) used in fix_time.
except ImportError:
    # Otherwise default to time module for non-embedded implementations
    # Should still support millisecond resolution.
    import time
    get_ticks = time.time


class MicropyGPS(object):
    """
    GPS NMEA sentence parser. Creates object that stores all relevant GPS data and statistics.
    Parses sentences one character at a time using update().
    """

    # Max number of characters a valid sentence can be (based on GGA sentence, 82 bytes incl. '$' and '\r\n')
    SENTENCE_LIMIT = 90
    #__HEMISPHERES = 'NSEW'
    __NO_FIX, __FIX_2D, __FIX_3D = 1, 2, 3
    __DIRECTIONS = ('N', 'NNE', 'NE', 'ENE', 'E', 'ESE', 'SE', 'SSE', 
                    'S', 'SSW', 'SW', 'WSW', 'W','WNW', 'NW', 'NNW')
    __MONTHS = ('January', 'February', 'March', 'April', 'May', 'June', 
                'July', 'August', 'September', 'October', 'November', 'December')
    CLEAR_DATE = (0, 0, 0)
    CLEAR_TIME = (0, 0, 0.0)
    CLEAR_LAT = (0, 0.0, 'N')
    CLEAR_LON = (0, 0.0, 'W')
    __f_nan = float('nan')

    def __init__(self, local_offset=0, location_formatting='ddm', century=None):
        """
        Setup GPS Object Status Flags, Internal Data Registers, etc
            local_offset (int): Timezone difference to UTC (preliminary support, date may be corrupted.)
                                I recommend using something else (e.g. datetime module).
            location_formatting (str): Style For Presenting Longitude/Latitude:
                                       Degrees Decimal Minutes (ddm) - 40° 26.767′ N
                                       Degrees Minutes Seconds (dms) - 40° 26′ 46″ N
                                       Decimal Degrees (dd) - 40.446° (negative values represent south/west)
            century (int): 19 or 20.  Updated if a GPZDA sentence is parsed.
        """

        #####################
        # Object Status Flags
        self.sentence_active = False
        self.active_segment = 0
        self.process_crc = False
        self.gps_segments = []
        self.__buf = bytearray() # Buffer for update()
        self.__buf_append = self.__buf.append
        self.crc_xor = 0
        self.char_count = 0
        self.fix_time = 0

        #####################
        # Sentence Statistics
        self.crc_fails = 0
        self.clean_sentences = 0
        self.parsed_sentences = 0

        #####################
        # Logging Related
        self.log_handle = None
        self.log_en = False

        #####################
        # Data From Sentences
        # Time and Date
        self.timestamp = self.CLEAR_TIME
        self.date = self.CLEAR_DATE
        self.century = century
        self.local_offset = local_offset

        # Position/Motion
        self._latitude = self.CLEAR_LAT
        self._longitude = self.CLEAR_LON
        self.coord_format = location_formatting
        self.speed = 0.0
        self.course = 0.0
        self.altitude = 0.0
        self.geoid_height = 0.0

        # GPS Info
        self.satellites_in_view = 0
        self.satellites_in_use = 0
        self.satellites_used = []
        self.last_sv_sentence = 0
        self.total_sv_sentences = 0
        self.satellite_data = dict()
        self.hdop = self.__f_nan
        self.pdop = self.__f_nan
        self.vdop = self.__f_nan
        self.valid = False
        self.fix_stat = 0 # Fix not available
        self.fix_type = self.__NO_FIX

    ########################################
    # Coordinates Translation Functions
    ########################################
    @property
    def latitude(self):
        """Format Latitude Data"""
        return self.__conv_lat_lon(self._latitude)

    @property
    def longitude(self):
        """Format Longitude Data"""
        return self.__conv_lat_lon(self._longitude)

    def __conv_lat_lon(self, lat_lon):
        """ Coordinate Format ('dd', 'dms', 'ddm') Conversions from 'ddm' 
        Returns a signed value in dd, a tuple in dms/ddm format.
        Positive and negative values in dd format indicate N/E and S/W, respectively.
        """
        if self.coord_format == 'dd':
            decimal_degrees = lat_lon[0] + (lat_lon[1] / 60)
            sign_dd = (lat_lon[2] in 'NE') - (lat_lon[2] in 'SW')
            return sign_dd * decimal_degrees
        elif self.coord_format == 'dms':
            return (
                lat_lon[0], # degrees
                int(lat_lon[1]), # minutes
                round((lat_lon[1] % 1) * 60), # seconds
                lat_lon[2], # hemisphere
            )
        return lat_lon # (degrees, decimal minutes, hemisphere)

    ########################################
    # Logging Related Functions
    ########################################
    def start_logging(self, target_file, mode="append"):
        """Create GPS data log object"""
        # Set write mode: Overwrite or Append
        mode_code = 'w' if mode == 'new' else 'a'

        try:
            self.log_handle = open(target_file, mode_code)
        except AttributeError:
            print("Invalid FileName")
            return False

        self.log_en = True
        return True

    def stop_logging(self):
        """Closes the log file handler and disables further logging"""
        try:
            self.log_handle.close()
        except AttributeError:
            print("Invalid Handle")
            return False

        self.log_en = False
        return True

    def write_log(self, log_string):
        """Attempts to write the last valid NMEA sentence character to the active file handler"""
        try:
            self.log_handle.write(log_string)
        except TypeError:
            return False
        return True

    ########################################
    # Sentence parsers
    ########################################
    def __parse_time(self, utc_string):
        """Parses timestamps from strings.
        Stores the parsed tuples (8, 18, 36.0) or (18, 0, 41.896)
        in self.timestamp.  Returns True if success.

        >>> __parse_time('081836')
        True
        >>> __parse_time('180041.896')
        True
        """

        # UTC timestamp
        if not utc_string:
            self.timestamp = self.CLEAR_TIME
            return True

        # Possible timestamp found, HHMMSS[.SSSSSS]
        try:
            hours = (int(utc_string[0:2]) + self.local_offset) % 24
            minutes = int(utc_string[2:4])
            seconds = float(utc_string[4:])

        except ValueError:  # Bad timestamp value present
            return False
        if seconds >= 60.0 or minutes >= 60:
            return False 

        # Update timestamp
        self.timestamp = (hours, minutes, seconds)
        return True

    def __parse_lat_lon(self, lat_str, lat_hemi, lon_str, lon_hemi):
        """Parses latitudes and logitudes from strings.
        Stores the parsed tuples (37, 51.65, 'S') and (145, 7.36, 'E'), respectively,
        in self._latitude and self._longitude.  Returns True if success.

        >>> __parse_lat_lon('3751.65', 'S', '14507.36', 'E')
        True
        """

        #if lat_hemi not in self.__HEMISPHERES or lon_hemi not in self.__HEMISPHERES:
        if lat_hemi not in "NS" or lon_hemi not in "EW":
            return False

        try:
            # Latitude: 'DDMM.MMMM'
            lat_degs = int(lat_str[0:2])
            lat_mins = float(lat_str[2:])
            # Longitude: 'DDDMM.MMMM'
            lon_degs = int(lon_str[0:3])
            lon_mins = float(lon_str[3:])
        except ValueError:
            return False

        self._latitude = (lat_degs, lat_mins, lat_hemi)
        self._longitude = (lon_degs, lon_mins, lon_hemi)
        return True

    def gprmc(self):
        """
        Parse Recommended Minimum Specific GPS/Transit data (RMC) Sentence.
        Updates UTC timestamp, latitude, longitude, course, speed, date, and fix status
        """

        # Check receiver data valid flag
        try:
            status = self.gps_segments[2]
        except IndexError:
            return False
        if status == 'A':  # Data from receiver is Valid/Has Fix

            # UTC timestamp
            if not self.__parse_time(self.gps_segments[1]):
                return False

            # Date stamp
            try:
                date_string = self.gps_segments[9]
                if date_string:  # Possible date stamp found
                    day = int(date_string[0:2])
                    month = int(date_string[2:4])
                    year = int(date_string[4:6])
                    self.date = (day, month, year)
                else:  # No Date stamp yet
                    self.date = self.CLEAR_DATE
            except (ValueError, IndexError):  # Bad date stamp value present
                return False

            # Latitude and Longitude 
            if not self.__parse_lat_lon(
                self.gps_segments[3], # latitude in 'DDMM.MMMM' format
                self.gps_segments[4], # hemisphere in 'N', 'S' 
                self.gps_segments[5], # longitude in 'DDDMM.MMMM' format
                self.gps_segments[6], # hemisphere in 'E', 'W'
            ): return False

            # Speed in knots
            try:
                spd_knt = float(self.gps_segments[7])
            except ValueError:
                return False

            # Course in degrees
            if not self.gps_segments[8]:
                course = 0.0
            else:
                try:
                    course = float(self.gps_segments[8])
                except ValueError:
                    return False

            # TODO - Add magnetic variation

            # Update object data
            self.speed = spd_knt
            self.course = course
            self.valid = True

            # Update last fix time
            self.new_fix_time()

        else:
            # Clear position data if sentence is 'Invalid'
            self._latitude = self.CLEAR_LAT
            self._longitude = self.CLEAR_LON
            self.speed = 0.0
            self.course = 0.0
            self.valid = False
            # Do we have to clear timestamp and date?
            #return True # Should it be False?

        return True

    def gpgll(self):
        """
        Parse Geographic Latitude and Longitude (GLL) Sentence. Updates UTC timestamp, latitude,
        longitude, and fix status
        """

        # Check receiver data valid flag
        try:
            status = self.gps_segments[6]
        except IndexError:
            return False
        if status == 'A':  # Data from receiver is Valid/Has Fix

            # UTC timestamp
            if not self.__parse_time(self.gps_segments[5]):
                return False

            # Latitude and Longitude
            if not self.__parse_lat_lon(
                self.gps_segments[1], # latitude in 'DDMM.MMMM' format
                self.gps_segments[2], # hemisphere in 'N', 'S' 
                self.gps_segments[3], # longitude in 'DDDMM.MMMM' format
                self.gps_segments[4], # hemisphere in 'E', 'W'
            ): return False

            # Update object data
            self.valid = True

            # Update last fix time
            self.new_fix_time()

        else:  # Clear position data if sentence is 'Invalid'
            self._latitude = self.CLEAR_LAT
            self._longitude = self.CLEAR_LON
            self.valid = False
            # Do we have to clear timestamp and date?
            #return True # Should it be False?

        return True

    def gpvtg(self):
        """Parse Track Made Good and Ground Speed (VTG) Sentence. Updates speed and course"""

        try:
            # Course in degrees
            course = float(self.gps_segments[1]) if self.gps_segments[1] else 0.0
            # Speed in knots
            spd_knt = float(self.gps_segments[5]) if self.gps_segments[5] else 0.0
        except (ValueError, IndexError):
            return False

        # Update object data
        self.speed = spd_knt
        self.course = course
        return True

    def gpgga(self):
        """
        Parse Global Positioning System Fix Data (GGA) Sentence. Updates UTC timestamp, latitude, longitude,
        fix status, satellites in use, Horizontal Dilution of Precision (HDOP), altitude, geoid height 
        and fix status
        """

        try:
            # Number of satellites in use
            satellites_in_use = int(self.gps_segments[7])
            # Get fix status
            fix_stat = int(self.gps_segments[6])
        except (ValueError, IndexError):
            return False

        # UTC timestamp
        if not self.__parse_time(self.gps_segments[1]):
            return False

        try:
            # Horizontal dilution of precision
            hdop = float(self.gps_segments[8])
        except (ValueError, IndexError):
            hdop = self.__f_nan

        # Process location data if fix is GOOD
        if fix_stat:

            # Latitude and Longitude
            if not self.__parse_lat_lon(
                self.gps_segments[2], # latitude in 'DDMM.MMMM' format
                self.gps_segments[3], # hemisphere in 'N', 'S' 
                self.gps_segments[4], # longitude in 'DDDMM.MMMM' format
                self.gps_segments[5], # hemisphere in 'E', 'W'
            ): return False

            # Altitude / Height Above Geoid
            try:
                altitude = float(self.gps_segments[9])
                geoid_height = float(self.gps_segments[11])
            except ValueError:
                altitude = 0.0
                geoid_height = 0.0

            # Update object data
            self.altitude = altitude
            self.geoid_height = geoid_height

            # Update last fix time
            self.new_fix_time()

        # Update object data
        self.satellites_in_use = satellites_in_use
        self.hdop = hdop
        self.fix_stat = fix_stat

        return True

    def gpgsa(self):
        """
        Parse GNSS DOP and Active Satellites (GSA) sentence. Updates GPS fix type, list of satellites used in
        fix calculation, Position Dilution of Precision (PDOP), Horizontal Dilution of Precision (HDOP), Vertical
        Dilution of Precision, and fix status
        """

        # Fix type (None,2D or 3D)
        try:
            fix_type = int(self.gps_segments[2])
        except ValueError:
            return False

        # Read All (up to 12) Available PRN Satellite Numbers
        sats_used = []
        for sats in range(12):
            sat_number_str = self.gps_segments[3 + sats]
            if sat_number_str:
                try:
                    sat_number = int(sat_number_str)
                    sats_used.append(sat_number)
                except ValueError:
                    return False
            else:
                break

        # PDOP,HDOP,VDOP
        try:
            pdop = float(self.gps_segments[15])
            hdop = float(self.gps_segments[16])
            vdop = float(self.gps_segments[17])
        except ValueError:
            return False

        # If fix is GOOD, update fix timestamp
        if fix_type > self.__NO_FIX:
            self.new_fix_time()

        # Update object data
        self.fix_type = fix_type
        self.satellites_used = sats_used
        self.hdop = hdop
        self.vdop = vdop
        self.pdop = pdop

        return True

    def gpgsv(self):
        """
        Parse Satellites in View (GSV) sentence. Updates number of SV Sentences, the number of 
        the last SV sentenceparsed, and data on each satellite present in the sentence
        """

        try:
            num_sv_sentences = int(self.gps_segments[1])
            current_sv_sentence = int(self.gps_segments[2])
            sats_in_view = int(self.gps_segments[3])
        except ValueError:
            return False

        # Create a blank dict to store all the satellite data from this sentence in:
        # satellite PRN is key, tuple containing telemetry is value
        satellite_dict = dict()

        # Calculate number of satelites to pull data for and thus how many segment positions to read
        if num_sv_sentences == current_sv_sentence:
            # Last sentence may have 1-4 satellites; 5 - 20 positions
            sat_segment_limit = (sats_in_view - ((num_sv_sentences - 1) * 4)) * 5
        else:
            sat_segment_limit = 20  # Non-last sentences have 4 satellites and thus read up to position 20

        # Try to recover data for up to 4 satellites in sentence
        for sats in range(4, sat_segment_limit, 4):

            try:
                # If no PRN is found, then the sentence has no more satellites to read
                if not self.gps_segments[sats]:
                   break
            except IndexError:
                return False

            # If a PRN is present, grab satellite data
            try:
                sat_id = int(self.gps_segments[sats])
            except (ValueError, IndexError):
                return False

            try:  # Elevation can be null (no value) when not tracking
                elevation = int(self.gps_segments[sats+1])
            except (ValueError, IndexError):
                elevation = None

            try:  # Azimuth can be null (no value) when not tracking
                azimuth = int(self.gps_segments[sats+2])
            except (ValueError, IndexError):
                azimuth = None

            try:  # SNR can be null (no value) when not tracking
                snr = int(self.gps_segments[sats+3])
            except (ValueError, IndexError):
                snr = None
            # Add satellite data to satellite_dict
            satellite_dict[sat_id] = (elevation, azimuth, snr)

        # Update object data
        self.total_sv_sentences = num_sv_sentences
        self.last_sv_sentence = current_sv_sentence
        self.satellites_in_view = sats_in_view

        # For a new set of sentences, we either clear out the existing sat data or
        # update it as additional SV sentences are parsed
        if current_sv_sentence == 1:
            self.satellite_data = satellite_dict
        else:
            self.satellite_data.update(satellite_dict)

        return True

    def gpzda(self):
        """
        Parse GPZDA sentence. Updates UTC timestamp, date and century.
        """

        # Date stamp and century
        try:
            day = int(self.gps_segments[2]) # 'DD'
            month = int(self.gps_segments[3]) # 'MM'
            str_year = self.gps_segments[4] # 'YYYY' format
            century, year = int(str_year[0:2]), int(str_year[2:4])

        except (ValueError, IndexError):  # Bad Date stamp value present
            self.date = self.CLEAR_DATE
            return False

        # UTC timestamp
        if not self.__parse_time(self.gps_segments[1]):
            return False

        # Update object data
        if self.century in (None, century - 1):
            self.century = century
        self.date = (day, month, year) # (DD, MM, YY)
        return True

    ##########################################
    # Data Stream Handler Functions
    ##########################################

    def new_sentence(self):
        """Adjust Object Flags in Preparation for a New Sentence"""
        self.gps_segments[:] = []
        self.active_segment = 0
        self.crc_xor = 0
        self.sentence_active = True
        self.process_crc = True
        self.char_count = 0
        self.__buf[:] = b''

    def __update_segment(self):
        self.gps_segments.append(self.__buf.decode('ascii'))
        self.__buf[:] = b''

    def update(self, new_char):
        """
        Process a new input char and updates GPS object if necessary based on special characters ('$', ',', '*')
        Function builds a list of received string that are validated by CRC prior to parsing by the appropriate
        sentence function.  Returns sentence type (e.g. 'GPRMC') on successful parse, None otherwise
        """

        # Validate new_char is a printable char. cf. SP := 0x20 = 32, LF := 0x0a = 10, CR := 0x0d = 13.
        ascii_char = ord(new_char)
        if 32 <= ascii_char <= 126 or ascii_char in (10, 13):
            self.char_count += 1

            # Write character to log file if enabled
            if self.log_en:
                self.write_log(new_char)

            # Check if a new sentence is starting ($)
            if ascii_char == 36: # '$' 36 = 0x24
                self.new_sentence()

            elif self.sentence_active:

                # Check if the active segment is ended (,), create a new segment to feed characters to
                if ascii_char == 44: # ',' 44 = 0x2c
                    self.__update_segment()
                    self.active_segment += 1

                # Check if the sentence is almost ending (*), CRC (2 bytes) follows
                elif ascii_char == 42: # '*' 42 = 0x2a
                    self.process_crc = False
                    self.__update_segment()
                    self.active_segment += 1
                    return None

                # Store all other printable character and check CRC when ready
                else:
                    self.__buf_append(ascii_char)

                # Update CRC (between the starting '$' and the ending '*' marks, excluding the marks.)
                if self.process_crc:
                    self.crc_xor ^= ascii_char

                # When CRC input is disabled sentence is nearly complete, additional 2 bytes necessary for CRC.
                elif len(self.__buf) == 2:
                    self.sentence_active = False  # Clear active processing flag
                    self.__update_segment() # Update CRC segment

                    try: # Check CRC errors
                        if self.crc_xor != int(self.gps_segments[self.active_segment], 16):
                            self.crc_fails += 1
                            return None
                    except ValueError:
                        # CRC Value was deformed and could not have been correct
                        return None
                    self.clean_sentences += 1  # Increment clean sentences received

                    # If the valid sentence is a supported sentence type, then parse it!!
                    if (self.gps_segments[0] in self.supported_sentences
                        and self.supported_sentences[self.gps_segments[0]](self)):
                    # Parse the sentence based on the message type, receive True if parse is clean
                        # Let host know that the GPS object was updated by returning parsed sentence type
                        self.parsed_sentences += 1
                        return self.gps_segments[0]

                # Avoid unsupported sentences to be processed. Can be controlled by supported_sentences (dict).
                # Also check that the sentence buffer isn't filling up with garbage waiting for the sentence 
                # to complete.
                if (self.active_segment == 1 and self.gps_segments[0] not in self.supported_sentences
                    or self.char_count > self.SENTENCE_LIMIT):
                    self.sentence_active = False

        # Tell host no new sentence was parsed
        return None

    def new_fix_time(self):
        """
        Updates a high resolution counter with current time when fix is updated. 
        Currently only triggered from GGA, GSA and RMC sentences
        """

        self.fix_time = get_ticks()

    #########################################
    # User Helper Functions
    # These functions make working with the GPS object data easier
    #########################################

    def satellite_data_updated(self):
        """
        Checks if all the GSV sentences in a group have been read, making satellite data complete
        :return: boolean
        """

        if self.total_sv_sentences > 0 and self.total_sv_sentences == self.last_sv_sentence:
            return True
        else:
            return False

    def unset_satellite_data_updated(self):
        """
        Mark GSV sentences as read indicating the data has been used and future updates are fresh
        """

        self.last_sv_sentence = 0

    def satellites_visible(self):
        """
        Returns a list of of the satellite PRNs currently visible to the receiver
        :return: list
        """

        return list(self.satellite_data.keys())

    def time_since_fix(self):
        """
        Returns number of millisecond since the last sentence with a valid fix was parsed. 
        Returns -1 if no fix has been found
        """

        # Test if a Fix has been found
        if self.fix_time == 0:
            return -1

        # Try calculating fix time using utime; if not running MicroPython
        # time.time() returns a floating point value in secs
        try:
            current = utime.ticks_diff(get_ticks(), self.fix_time)
        except NameError:
            current = (get_ticks() - self.fix_time) * 1000  # ms
        return current

    def compass_direction(self):
        """
        Determine a cardinal or inter-cardinal direction based on current course.
        :return: string
        """

        # Calculate the offset for a rotated compass
        offset_course = (self.course + 11.25) % 360.0
        # Each compass point is separated by 22.5 degrees, divide to find lookup value
        return self.__DIRECTIONS[int(offset_course // 22.5)]

    def __pp_lat_lon(self, lat_lon):
        """
        Prettify latitude/longitude strings.
        :return: string
        """

        deg, min, sec = '°', "'", '"'
        if self.coord_format == 'dd':
            return f'{lat_lon}{deg}'
        elif self.coord_format == 'dms':
            d, m, s, hemi = lat_lon
            return f'{d}{deg} {m}{min} {s}{sec} {hemi}'
        else:
            d, dm, hemi = lat_lon
            return f'{d}{deg} {dm}{min} {hemi}'

    def latitude_string(self):
        """
        Create a readable string of the current latitude data
        :return: string
        """
        return self.__pp_lat_lon(self.latitude)

    def longitude_string(self):
        """
        Create a readable string of the current longitude data
        :return: string
        """
        return self.__pp_lat_lon(self.longitude)

    def speed_string(self, unit='kph'):
        """
        Creates a readable string of the current speed data in one of three units
        :param unit: string of 'kph','mph, or 'knot'
        :return: string
        """

        if unit == 'mph':
            spd = self.speed * 1.151
        elif unit == 'knot':
            if self.speed != 1:
                unit = 'knots'
            spd = self.speed
        else:
            unit = 'km/h'
            spd = self.speed * 1.852
        return f'{spd} {unit}'

    def date_string(self, formatting='s_mdy', century=None):
        """
        Creates a readable string of the current date.
        Can select between long format: Januray 1st, 2014 or two short formats:
        11/01/2014 (MM/DD/YYYY), 01/11/2014 (DD/MM/YYYY).
        :param formatting: string 'long', 's_mdy' or 's_dmy' 
        :param century: int (or str) delineating the century the GPS data is from (19 for 19XX, 20 for 20XX)
        :return: date_string with the specified format
        """

        century = century or self.century or 20
        # Create Year String.  Add leading zeros to year string if necessary.
        year = f'{century}{self.date[2]:02d}'

        # Long Format
        if formatting == 'long':
            # Retrieve month string from private set
            month = self.__MONTHS[self.date[1] - 1]
            # Determine date suffix and create day strings
            suffix = ('th', 'st', 'nd', 'rd')
            index = self.date[0] % 10
            if not 1 <= index <= 3:
                index = 0
            day = f'{self.date[0]}{suffix[index]}'
            # Put it all together
            date_string = f'{month} {day}, {year}'

        else:
            # Add leading zeros to day string if necessary
            day = f'{self.date[0]:02d}'
            # Add leading zeros to month string if necessary
            month = f'{self.date[1]:02d}'
            # Build final string based on desired formatting
            if formatting == 's_dmy':
                date_string = f'{day}/{month}/{year}'
            else:  # Default date format: 's_mdy'
                date_string = f'{month}/{day}/{year}'

        return date_string

    # All the currently supported NMEA sentences, can be used to limit the sentence type to parse
    supported_sentences = {'GPRMC': gprmc, 'GLRMC': gprmc,
                           'GPGGA': gpgga, 'GLGGA': gpgga,
                           'GPVTG': gpvtg, 'GLVTG': gpvtg,
                           'GPGSA': gpgsa, 'GLGSA': gpgsa,
                           'GPGSV': gpgsv, 'GLGSV': gpgsv,
                           'GPGLL': gpgll, 'GLGLL': gpgll,
                           'GNGGA': gpgga, 'GNRMC': gprmc,
                           'GNVTG': gpvtg, 'GNGLL': gpgll,
                           'GNGSA': gpgsa, 'GPZDA': gpzda,
                          }

if __name__ == "__main__":
    pass
