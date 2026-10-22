import time, collections, gc, asyncio, vfs, os
from alt_micropyGPS import MicropyGPS
from machine import Pin, UART, I2C, SPI
import machine
import ssd1306, sdcard
import datetime
import micropython
import rp2

micropython.alloc_emergency_exception_buf(100)

__str_array = const(b'ABCDEFGHIJKLMNOPQRSTUVWXYZ') #For GridLocator Calc

# Lock And EventFlag for asyncio
gnss_newchar = asyncio.Event()
gnss_logstart = asyncio.Event()
gnss_logstop = asyncio.Event()
gnss_logstopped = asyncio.Event()
readbuf_lock = asyncio.Lock()
oled_lock = asyncio.Lock()
gnss_updatenow = asyncio.Event()
pps_irq_flag = asyncio.ThreadSafeFlag()
logging_stop_flag = asyncio.ThreadSafeFlag()
Card_Inserted = asyncio.Event()

#datetime service variables
td_jst_wDelay = datetime.timedelta(hours=9, seconds=1) # add 1sec Delay for display

# Running Mode Config
config_atFirst = False
uart_outputnmea = False
highspeed_logging = True # Fix rate 10Hz (Display is still updated at 1Hz)
FIX_RATE_HZ = 10 if highspeed_logging else 1
BUFFSIZE = const(1024*4)
UART_RXBUF = BUFFSIZE if highspeed_logging else BUFFSIZE>>1
UART_READ_CHUNK = const(512)
GNSS_BAUD = const(230400) # UART baudrate of the GNSS module
GNSS_BAUD_OLD = const(115200) # Previous baudrate, used only for config_atFirst
GNSS_BAUD_CMD = '$PSTMCFGPORT,0,0,2,230400*76\r\n' # Must match GNSS_BAUD
CPU_FREQ = const(150_000_000) # RP2350 default is 150MHz
debug_timing = False # Print processing time statistics every second

# Logging state
log_stopped_by_button = False

#Tweak MicropyGPS GSV Parser
MicropyGPS.supported_sentences['GNGSV'] = MicropyGPS.gpgsv

# Timing measurement (debug_timing). Times are in us, max values in the last 1 second
class TimingStat:
    def __init__(self):
        self.ts_pps = (0, 0, 0.0) # gnss.timestamp at the PPS
        self.reset()

    def reset(self):
        self.backlog = 0   # max uart.any() before read [byte]
        self.read_max = 0  # max time of uart.read()
        self.nbytes = 0    # total parsed bytes
        self.parse_max = 0 # max time of parsing one read
        self.parse_sum = 0 # total time of parsing
        self.wlog_max = 0  # max time of write_log
        self.wlog_slow = 0 # count of write_log over 10ms
        self.flush_max = 0 # max time of flush_log
        self.draw_max = 0  # max time of drawing frame buffer
        self.show_max = 0  # max time of oled.show()
        self.gc_max = 0    # max time of gc.collect()
        self.loop_blk = 0  # max blocking time of the event loop

tstat = TimingStat()

if debug_timing:
    _write_log_orig = MicropyGPS.write_log
    def _write_log_timed(self, log_string):
        t = time.ticks_us()
        r = _write_log_orig(self, log_string)
        dt = time.ticks_diff(time.ticks_us(), t)
        if dt > tstat.wlog_max:
            tstat.wlog_max = dt
        if dt > 10_000:
            tstat.wlog_slow += 1
        return r
    MicropyGPS.write_log = _write_log_timed

# Pin ISR for Syncronize to PPS signal
def detectPPS(Pin):
    pps_irq_flag.set()

def removeCard(Pin):
    logging_stop_flag.set()

# Pin and IRQ setting
gnss_resetn = Pin(6, Pin.OUT)
fix_led = Pin("LED", Pin.OUT)
status_led = Pin(7, Pin.OUT)
pps_pin = Pin(22, Pin.IN)
pps_pin.irq(trigger=Pin.IRQ_RISING, handler=detectPPS)
det_pin = Pin(9, Pin.IN) 
det_pin.irq(trigger=Pin.IRQ_RISING, handler=removeCard)

fix_led.value(0)
status_led.value(1 if highspeed_logging else 0) # Lit when the fix rate is high

# global variables
logfile_str = ''
sg = bytearray(8)

async def uart_readgnss(uart: UART, gnss: MicropyGPS):
    global logfile_str
    while True:
        n = uart.any()
        if n:
            if n > tstat.backlog:
                tstat.backlog = n
            t = time.ticks_us()
            # Read only the received bytes. uart.read() without size waits timeout_char for the next char,
            # and the chunk size is limited to give other tasks a chance to run.
            s = uart.read(min(n, UART_READ_CHUNK))
            dt = time.ticks_diff(time.ticks_us(), t)
            if dt > tstat.read_max:
                tstat.read_max = dt
            t = time.ticks_us()
            if s is not None:
                if uart_outputnmea:
                    try:
                        str_utf = str(s, 'UTF-8', "")
                        print(str_utf, end="")
                    except Exception as e:
                        print(e)
                if gnss_logstop.is_set():
                    gnss.stop_logging()
                    gnss_logstop.clear()
                    gnss_logstopped.set()
                    print('logging stopped')
                gnss.update_bytes(s) # line by line parsing, much faster than gnss.update() for each char
                if gnss_logstart.is_set():
                    gnss.start_logging(logfile_str, mode='new')
                    print(f'Logging Start with filename {logfile_str}')
                    gnss_logstart.clear()
                gnss_updatenow.set()
                dt = time.ticks_diff(time.ticks_us(), t)
                tstat.nbytes += len(s)
                tstat.parse_sum += dt
                if dt > tstat.parse_max:
                    tstat.parse_max = dt
        else:
            await asyncio.sleep_ms(10)
            continue
        await asyncio.sleep(0)

def lat_lon_string(lat_lon):
    min = "'"
    d, dm, hemi = lat_lon
    return f'{d:3d} {dm:06.03f}{min}{hemi}'


def gridlocator_calc(lat, lon):
    global sg
    d_lat, dm_lat, hemi = lat
    d_lon, dm_lon, hemi = lon
    gf_lat = (d_lat + dm_lat/60 + 90)/10
    gf_lon = (d_lon + dm_lon/60 + 180)/20
    gf_lat_int = int(gf_lat)
    gf_lon_int = int(gf_lon)
    sg[0] = __str_array[gf_lon_int]
    sg[1] = __str_array[gf_lat_int]
    gf_lat = (gf_lat - gf_lat_int)*10
    gf_lon = (gf_lon - gf_lon_int)*10
    gf_lat_int = int(gf_lat)
    gf_lon_int = int(gf_lon)
    sg[2] = gf_lon_int + 0x30
    sg[3] = gf_lat_int + 0x30
    gf_lat = (gf_lat - gf_lat_int)*24
    gf_lon = (gf_lon - gf_lon_int)*24
    gf_lat_int = int(gf_lat)
    gf_lon_int = int(gf_lon)
    sg[4] = __str_array[gf_lon_int]
    sg[5] = __str_array[gf_lat_int]
    gf_lat = (gf_lat - gf_lat_int)*10
    gf_lon = (gf_lon - gf_lon_int)*10
    gf_lat_int = int(gf_lat)
    gf_lon_int = int(gf_lon)
    sg[6] = gf_lon_int + 0x30
    sg[7] = gf_lat_int + 0x30
    #print(str(sg, 'UTF-8'))
    return sg

def datetime_toJST(gnss_date:tuple[int,int,int], timestamp:tuple[int, int, float], offset:datetime.timedelta):
    #day = int(gnss_date[0])
    #month = int(gnss_date[1])
    #year = int(gnss_date[2])+2000
    hour, minutes, seconds = timestamp
    dt_now_jst = datetime.datetime(
        int(gnss_date[2])+2000,
        int(gnss_date[1]),
        int(gnss_date[0]),
        hour,minutes,int(seconds),0,
        datetime.timezone.utc)+offset
    
    dateobj = dt_now_jst.date()
    #datestr = f'{dateobj.year-2000:02d}{dateobj.month:02d}{dateobj.day:02d}'
    timeobj = dt_now_jst.time()
    dt_now_jst_str = f'{dateobj.year-2000:02d}{dateobj.month:02d}{dateobj.day:02d} {timeobj.hour:02d}:{timeobj.minute:02d}:{timeobj.second:02d}'
    return dt_now_jst_str

def datetime_toJST_filestr(gnss_date:tuple[int,int,int], timestamp:tuple[int, int, float], offset:datetime.timedelta):
    #day = int(gnss_date[0])
    #month = int(gnss_date[1])
    #year = int(gnss_date[2])+2000
    hour, minutes, seconds = timestamp
    dt_now_jst = datetime.datetime(
        int(gnss_date[2])+2000,
        int(gnss_date[1]),
        int(gnss_date[0]),
        hour,minutes,int(seconds),0,
        datetime.timezone.utc)+offset
    dateobj = dt_now_jst.date()
    #datestr = f'{dateobj.year-2000:02d}{dateobj.month:02d}{dateobj.day:02d}'
    timeobj = dt_now_jst.time()
    dt_now_jst_str = f'{dateobj.year-2000:02d}{dateobj.month:02d}{dateobj.day:02d}_{timeobj.hour:02d}{timeobj.minute:02d}{timeobj.second:02d}.nmea'
    return dt_now_jst_str

async def display_update(gnss: MicropyGPS, oled:ssd1306.SSD1306_I2C):
    dt_now = '000000 00:00:00'
    lat = '000 00.000''N'
    lon = '000 00.000''E'
    gl = b'XX00XX00'

    last_sec = -1
    last_draw = time.ticks_ms()

    fix_led.value(0)
    while True:
        await gnss_updatenow.wait()
        gnss_updatenow.clear()
        if highspeed_logging:
            # Draw once per UTC second; oled.show() runs on the next PPS.
            # Do not wait for the hh:mm:ss.0 epoch: several epochs can be parsed at once
            # (e.g. after SD card writing), then the .0 epoch is not seen here.
            sec = int(gnss.timestamp[2])
            if sec == last_sec and time.ticks_diff(time.ticks_ms(), last_draw) < 1000:
                continue
            last_sec = sec
            last_draw = time.ticks_ms()
        t = time.ticks_us()
        if gnss.fix_type == 1:
            fix_led.toggle()
        else:
            fix_led.value(1)
            #day = f'{gnss.date[0]:02d}'
            #month = f'{gnss.date[1]:02d}'
            #year = f'{gnss.date[2]:02d}'
            #date_str = f'{gnss.date[2]:02d}/{gnss.date[1]:02d}/{gnss.date[0]:02d}'
            lat = lat_lon_string(gnss.latitude)
            lon = lat_lon_string(gnss.longitude)
            gl = gridlocator_calc(gnss.latitude, gnss.longitude)
            if gnss.valid:
                dt_now = datetime_toJST(gnss.date,gnss.timestamp, td_jst_wDelay)
        oled.fill(0)
        #QZSS_prns = [x for x in gnss.satellites_used if x >= 184]
        await oled_lock.acquire()
        oled.text(f'Lat:{lat}', 0, 0)
        oled.text(f'Lon:{lon}', 0, 8)
        oled.text(dt_now, 8, 16)
        oled.text(f'FIX:{gnss.fix_type}  Sat:{gnss.satellites_in_use:02d}/{gnss.satellites_in_view:02d}',0,24)
        oled.text(f'HDOP:{gnss.hdop: 2.1f}',0,32)
        oled.text(f'{FIX_RATE_HZ:2d}Hz',96,32) # Fix rate, right aligned
        oled.text('GRID: '+str(gl, 'UTF-8'),0,40)
        oled.text(f'CRC NG:{gnss.crc_fails: 9d}',0,48)
        #oled.text(f'Elaps:{gnss.time_since_fix:10d}', 48,0)
        oled.text(f'|   |',0, 56)
        if gnss.log_en is True:
            if gnss.log_charnum < 10*1024*1024:
                oled.text(f'|Log|{gnss.log_charnum/1024: 9.1f}KB',0, 56)
            else: # Over 10MB
                oled.text(f'|Log|{gnss.log_charnum/(1024*1024): 9.2f}MB',0, 56)
        elif log_stopped_by_button:
            oled.text(f'|Stp| Remove SD',0, 56)
        oled_lock.release()
        dt = time.ticks_diff(time.ticks_us(), t)
        if dt > tstat.draw_max:
            tstat.draw_max = dt

async def display_sync(oled:ssd1306.SSD1306_I2C, gnss: MicropyGPS):
    while True:
        await pps_irq_flag.wait()
        tstat.ts_pps = gnss.timestamp
        await oled_lock.acquire()
        t = time.ticks_us()
        oled.show()
        dt = time.ticks_diff(time.ticks_us(), t)
        oled_lock.release()
        pps_irq_flag.clear()
        if dt > tstat.show_max:
            tstat.show_max = dt

async def loop_monitor():
    # Measure how long the event loop is blocked (oversleep of 10ms sleep)
    while True:
        t = time.ticks_us()
        await asyncio.sleep_ms(10)
        dt = time.ticks_diff(time.ticks_us(), t) - 10_000
        if dt > tstat.loop_blk:
            tstat.loop_blk = dt

async def timing_report(gnss: MicropyGPS, uart: UART):
    while True:
        await asyncio.sleep(1)
        h, m, s = tstat.ts_pps
        # MicroPython does not support concatenation of adjacent f-strings with braces
        print(f'[T] pps@{h:02d}:{m:02d}:{s:04.1f} now:{uart.any():4d}B bl:{tstat.backlog:4d}B' +
              f' rx:{tstat.nbytes:5d}B read:{tstat.read_max/1000:5.1f}ms' +
              f' parse:{tstat.parse_max/1000:5.1f}/{tstat.parse_sum/1000:4.0f}ms' +
              f' wlog:{tstat.wlog_max/1000:5.1f}ms({tstat.wlog_slow})' +
              f' flush:{tstat.flush_max/1000:5.1f} draw:{tstat.draw_max/1000:5.1f}' +
              f' show:{tstat.show_max/1000:5.1f} gc:{tstat.gc_max/1000:5.1f}' +
              f' blk:{tstat.loop_blk/1000:6.1f}ms crc:{gnss.crc_fails} log:{int(gnss.log_en)}')
        tstat.reset()

async def gc_coro():
    gc.enable()
    while True:
        t = time.ticks_us()
        gc.collect()
        dt = time.ticks_diff(time.ticks_us(), t)
        if dt > tstat.gc_max:
            tstat.gc_max = dt
        gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
        # for memory debug
        #print("mem_alloc:", gc.mem_alloc())
        #print("mem_free:", gc.mem_free())
        #print("mem_info:")
        #print(micropython.mem_info())
        await asyncio.sleep(5)

async def log_flush(gnss:MicropyGPS):
    # Log file is kept open while logging, so sync it periodically to keep the data on card removal
    while True:
        await asyncio.sleep(2)
        if gnss.log_en is True:
            try:
                t = time.ticks_us()
                gnss.flush_log()
                dt = time.ticks_diff(time.ticks_us(), t)
                if dt > tstat.flush_max:
                    tstat.flush_max = dt
            except OSError as e:
                print('flush_log: ', e)

async def bootsel_watch(gnss:MicropyGPS):
    # BOOTSEL button has no IRQ, so poll it
    while True:
        if rp2.bootsel_button() == 1:
            if gnss.log_en is True:
                logging_stop_flag.set()
            while rp2.bootsel_button() == 1:
                await asyncio.sleep_ms(50)
        await asyncio.sleep_ms(100)

async def card_detect_and_loggingCtrl(gnss:MicropyGPS):
    global logfile_str, log_stopped_by_button
    while True:
        if det_pin.value() == 0:
            Card_Inserted.set()
            await asyncio.sleep(1)
            if det_pin.value() == 1:
                Card_Inserted.clear()
                continue
            if gnss.fix_type == MicropyGPS.__FIX_3D and gnss.valid is True:
                spi = SPI(1, baudrate=100_000, sck=Pin(10), mosi=Pin(11), miso=Pin(12))
                sd = sdcard.SDCard(spi, Pin(15), baudrate=20_000_000)
                await asyncio.sleep_ms(100)
                vfs.mount(sd, '/sd')
                os.chdir('/sd')
                if 'LOG' not in os.listdir('/sd'):
                    os.mkdir('LOG')
                    print('/sd/LOG directory make')
                if gnss.valid:
                    dt_str = datetime_toJST_filestr(gnss.date,gnss.timestamp, td_jst_wDelay)
                logfile_str = '/sd/LOG/'+dt_str
                print(logfile_str)
                logging_stop_flag.clear() # discard stale stop request
                gnss_logstart.set()
            else:
                await asyncio.sleep(1)
                continue

            await logging_stop_flag.wait()
            gnss.log_en = False
            gnss_logstop.set()
            await gnss_logstopped.wait()
            vfs.umount('/sd')
            del sd
            del spi
            if det_pin.value() == 0:
                # Stopped by BOOTSEL button: do not restart until the card is removed
                log_stopped_by_button = True
                print('Logging stopped by button. Remove the card.')
                while det_pin.value() == 0:
                    await asyncio.sleep_ms(200)
                log_stopped_by_button = False
            Card_Inserted.clear()
            print('Card is Removed!')
            await asyncio.sleep(1)
        elif det_pin.value() == 1:
            Card_Inserted.clear()
            await asyncio.sleep(2)


async def gnss_read(uart: UART, gnss: MicropyGPS):
    gc.collect()

    i2c=I2C(1,sda=Pin(18),scl=Pin(19),freq=400000)
    oled=ssd1306.SSD1306_I2C(128,64,i2c)
    uart.init(baudrate=GNSS_BAUD, tx=Pin(0,Pin.OUT), rx=Pin(1, Pin.IN), timeout_char =16, rxbuf=UART_RXBUF)
    tasks = [
        uart_readgnss(uart, gnss),
        display_update(gnss, oled),
        display_sync(oled, gnss),
        card_detect_and_loggingCtrl(gnss),
        bootsel_watch(gnss),
        log_flush(gnss),
        gc_coro()
    ]
    if debug_timing:
        tasks.append(loop_monitor())
        tasks.append(timing_report(gnss, uart))
    await asyncio.gather(*tasks)


def gnss_configure(uart: UART):
    uart.write('$PSTMCFGCONST,2,2,2,2,0*01\r\n') # Set Positioning Constelation to GPS+GLONASS+GALILEO+QZSS
    uart.write('$PSTMSBASSERVICE,15*6C\r\n') # Set SBAS Service Auto
    uart.write('$PSTMSTAGPSSETCONSTMASK,3*14\r\n') # Use STAGPS for GPS and GLONASS
    uart.write('$PSTMSTAGPSONOFF,1*4B\r\n') # Use STAGPS Autonomous AGPS
    uart.write('$PSTMSETPAR,1130,0,2*36\r\n') # Set CPU Frequency to 196MHz
    uart.write('$PSTMSETPAR,1200,4,1*31\r\n') # Use SBAS Service
    uart.write('$PSTMSETPAR,1200,80000,1*3D\r\n') # GSV sentence talker ID change to GN Only
    if highspeed_logging:
        uart.write('$PSTMSETPAR,1303,0.1*35\r\n') # Set Fix Rate to 10Hz
        uart.write('$PSTMSETPAR,1201,80000,1*3C\r\n') # Enable GSV sentence (OR 0x80000)
        uart.write('$PSTMSETPAR,1201,100000,2*06\r\n') # Disable GLL sentence (AND NOT 0x100000)
    else:
        uart.write('$PSTMSETPAR,1303,1.0*35\r\n') # Set Fix Rate to 1Hz
        uart.write('$PSTMSETPAR,1201,180000,1*0D\r\n') # Enable GSV and GLL sentence (OR 0x180000)
    uart.write(GNSS_BAUD_CMD) # Set UART baudrate (effective after reset)
    uart.write('$PSTMSAVEPAR*58\r\n') # Save Parameters
    uart.write('$PSTMSRR*49\r\n') # Software Reset
    time.sleep_ms(200)
    if uart.any():
        s = uart.read()
        try:
            str_utf = str(s, 'UTF-8', "")
            print(str_utf, end="")
        except Exception as e:
            print(e)

def main():
    # Change CPU clock before initializing UART/I2C/SPI (their baudrates depend on the clock)
    machine.freq(CPU_FREQ)
    print(f'CPU Freq: {machine.freq()//1_000_000}MHz')

    gnss_resetn.value(0)
    time.sleep_ms(100)
    gnss_resetn.value(1)

    time.sleep_ms(1000)
    
    uart = UART(0, baudrate=GNSS_BAUD, tx=Pin(0,Pin.OUT), rx=Pin(1, Pin.IN), timeout_char =16, rxbuf=255)

    gnss_reciever = MicropyGPS()

    if config_atFirst:
        print('Configuring GNSS Reciever...')
        # The module may still work at the old baudrate, so send the config at both baudrates.
        # New baudrate becomes effective after $PSTMSAVEPAR and $PSTMSRR.
        for baud in (GNSS_BAUD_OLD, GNSS_BAUD):
            print(f'Send config at {baud}bps')
            uart.init(baudrate=baud, tx=Pin(0,Pin.OUT), rx=Pin(1, Pin.IN), timeout_char =16, rxbuf=255)
            gnss_configure(uart)
            time.sleep_ms(3000) # Wait for the module restart

        time.sleep_ms(2000)

    asyncio.run(gnss_read(uart, gnss_reciever))
    return -1

if __name__ == '__main__':
    main()
