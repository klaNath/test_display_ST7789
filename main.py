import math, time, collections, gc, asyncio
from micropyGPS import MicropyGPS
from machine import Pin, UART, I2C, RTC
import machine
import ssd1306
import datetime
import micropython

micropython.alloc_emergency_exception_buf(100)

__str_array = bytes(b'ABCDEFGHIJKLMNOPQRSTUVWXYZ')

gnss_newchar = asyncio.Event()
readbuf_lock = asyncio.Lock()
oled_lock = asyncio.Lock()
gnss_updatenow = asyncio.Event()
pps_irq_flag = asyncio.ThreadSafeFlag()
rtc = RTC()
td_jst = datetime.timedelta(hours=9)

MicropyGPS.supported_sentences['GNGSV'] = MicropyGPS.gpgsv

def detectPPS(Pin):
    pps_irq_flag.set()

gnss_resetn = Pin(6, Pin.OUT)
pps_pin = Pin(22, Pin.IN)
pps_pin.irq(trigger=Pin.IRQ_RISING, handler=detectPPS)

async def uart_readgnss(uart: UART,buf: collections.deque):
    while True:
        if uart.any():
            s = uart.read()
            try:
                str_utf = str(s, 'UTF-8', "")
                print(str_utf, end="")
            except Exception as e:
                print(e)

            if s is not None:
                await readbuf_lock.acquire()
                buf.extend(bytearray(s))
                readbuf_lock.release()
                gnss_newchar.set()
        await asyncio.sleep_ms(10)

async def gnss_update(gnss: MicropyGPS, buf: collections.deque):
    while True:
        await gnss_newchar.wait()
        await readbuf_lock.acquire()
        while len(buf) != 0:
            gnss.update(chr(buf.popleft()))
        gnss_updatenow.set()
        gnss_newchar.clear()
        readbuf_lock.release()

def lat_lon_string(lat_lon):
    min = "'"
    d, dm, hemi = lat_lon
    return f'{d:3d} {dm:06.03f}{min}{hemi}'

def gridlocator_calc(lat, lon):
    sg = bytearray()
    d_lat, dm_lat, hemi = lat
    d_lon, dm_lon, hemi = lon
    gf_lat = (d_lat + dm_lat/60 + 90)/10
    gf_lon = (d_lon + dm_lon/60 + 180)/20
    sg.append(__str_array[math.floor(gf_lon)])
    sg.append(__str_array[math.floor(gf_lat)])
    gf_lat = (gf_lat - math.floor(gf_lat))*10
    gf_lon = (gf_lon - math.floor(gf_lon))*10
    sg.append(math.floor(gf_lon) + 0x30)
    sg.append(math.floor(gf_lat) + 0x30)
    gf_lat = (gf_lat - math.floor(gf_lat))*24
    gf_lon = (gf_lon - math.floor(gf_lon))*24
    sg.append(__str_array[math.floor(gf_lon)])
    sg.append(__str_array[math.floor(gf_lat)])
    gf_lat = (gf_lat - math.floor(gf_lat))*10
    gf_lon = (gf_lon - math.floor(gf_lon))*10
    sg.append(math.floor(gf_lon) + 0x30)
    sg.append(math.floor(gf_lat) + 0x30)
    #print(str(sg, 'UTF-8'))
    return sg

def datetime_toJST(gnss_date, timestamp, offset):
    day = int(gnss_date[0])
    month = int(gnss_date[1])
    year = int(gnss_date[2])+2000
    hour, minutes, seconds = timestamp
    #rtc.datetime((year, month, day, 0, hour, minutes, int(seconds), 0))
    dt_now_jst = datetime.datetime(year, month, day, hour,minutes,int(seconds),0,datetime.timezone.utc)+td_jst
    #print(dt_now_jst)
    dateobj = dt_now_jst.date()
    datestr = f'{dateobj.year-2000:02d}{dateobj.month:02d}{dateobj.day:02d}'
    timeobj = dt_now_jst.time()
    dt_now_jst_str = f'{datestr} {timeobj.hour:02d}:{timeobj.minute:02d}:{timeobj.second:02d}'
    return dt_now_jst_str

async def display_update(gnss: MicropyGPS, oled:ssd1306.SSD1306_I2C):
    dt_now = '000000 00:00:00'
    lat = '000 00.000''N'
    lon = '000 00.000''E'
    gl = b'XX00XX00'
    fix_led = Pin("LED", Pin.OUT)
    fix_led.value(0)
    while True:
        await gnss_updatenow.wait()
        if gnss.fix_type == 1:
            fix_led.toggle()
            #print('No Fix')
        else:
            fix_led.value(1)
            day = f'{gnss.date[0]:02d}'
            month = f'{gnss.date[1]:02d}'
            year = f'{gnss.date[2]:02d}'
            date_str = f'{year}/{month}/{day}'
            lat = lat_lon_string(gnss.latitude)
            lon = lat_lon_string(gnss.longitude)
            gl = gridlocator_calc(gnss.latitude, gnss.longitude)
            dt_now = datetime_toJST(gnss.date,gnss.timestamp,offset=9)
            #print('UTC Timestamp:', gnss.timestamp)
            #print('Date:', date_str)
            #print(dt_now)
            #print('Lat:', lat, sep='')
            #print('Lon:', lon, sep='')
            #print('Horizontal Dilution of Precision:', gnss.hdop)
            #print()
        oled.fill(0)
        #QZSS_prns = [x for x in gnss.satellites_used if x >= 184]
        await oled_lock.acquire()
        oled.text(f'Lat:{lat}', 0, 0)
        oled.text(f'Lon:{lon}', 0, 8)
        oled.text(dt_now, 8, 16)
        oled.text(f'FIX:{gnss.fix_type} Sat:{gnss.satellites_in_use:02d}/{gnss.satellites_in_view:02d}',0,24)
        oled.text(str(gl, 'UTF-8'), 32, 48)
        oled_lock.release()
        gnss_updatenow.clear()
        
        #oled.show()
        await asyncio.sleep_ms(500)

async def display_sync(oled:ssd1306.SSD1306_I2C):
    while True:
        await pps_irq_flag.wait()
        state = machine.disable_irq()
        await oled_lock.acquire()
        oled.show()
        oled_lock.release()
        pps_irq_flag.clear()
        machine.enable_irq(state)


async def gnss_read(uart: UART, gnss: MicropyGPS):
    gc.collect()

    i2c=I2C(1,sda=Pin(18),scl=Pin(19),freq=400000)
    oled=ssd1306.SSD1306_I2C(128,64,i2c)

    buf = collections.deque('', 1024*4)
    uart.init(baudrate=9600, tx=Pin(0,Pin.OUT), rx=Pin(1, Pin.IN), timeout_char =16, rxbuf=1024*2)
    await asyncio.gather(
        uart_readgnss(uart, buf),
        gnss_update(gnss, buf),
        display_update(gnss, oled),
        display_sync(oled)
    )


def main():
    gnss_resetn.value(0)
    time.sleep_ms(100)
    gnss_resetn.value(1)

    time.sleep_ms(1000)
    uart = UART(0, baudrate=9600, tx=Pin(0,Pin.OUT), rx=Pin(1, Pin.IN), timeout_char =16, rxbuf=255)

    gnss_reciever = MicropyGPS()

    config_atFirst = False
    if config_atFirst: 
        uart.write('$PSTMCFGCONST,2,2,2,2,0*01\r\n')
        uart.write('$PSTMSBASSERVICE,15*6C\r\n')
        uart.write('$PSTMSTAGPSSETCONSTMASK,3*14\r\n')
        uart.write('$PSTMSTAGPSONOFF,1*4B\r\n')
        uart.write('$PSTMSETPAR,1200,4,1*31\r\n')
        uart.write('$PSTMSETPAR,1200,80000,1*3D\r\n')
        uart.write('$PSTMSAVEPAR*58\r\n')
        uart.write('$PSTMSRR*49\r\n')
        time.sleep_ms(200)
        if uart.any():
            s = uart.read()
            try:
                str_utf = str(s, 'UTF-8', "")
                print(str_utf, end="")
            except Exception as e:
                print(e)

        time.sleep_ms(5000)

    asyncio.run(gnss_read(uart, gnss_reciever))
    return -1

if __name__ == '__main__':
    main()
