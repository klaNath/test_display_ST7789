from machine import I2C,Pin
from utime import sleep
import ssd1306

i2c=I2C(1,sda=Pin(18),scl=Pin(19),freq=400000)
oled=ssd1306.SSD1306_I2C(128,64,i2c)

oled.text("Hello World!", 0, 5)
oled.show()