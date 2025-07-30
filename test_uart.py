from machine import Pin, I2C, UART
import time

# UARTの初期化です。
# UARTの番号とボーレートを指定します。
uart = UART(0, 9600,tx=Pin(0), rx=Pin(1))



gnss_resetn = Pin(6, Pin.OUT)

gnss_resetn.value(0)
time.sleep_ms(1000)
gnss_resetn.value(1)

# init関数はエラーになるので使用できません。
#uart.init(115200,bits=8, parity=None, stop=1 )

#
# データの送信処理です
# 注意：末尾に「\r」をつけないと、データが送信されません。
#
#uart.write("12345\r")
#print("write done")

#
# データの受信処理です
# 1byteずつ読み込んで表示しています。
#
print("read start")

for i in range(1000):
    buf = uart.read(1)
    print(buf)
    
    time.sleep_ms(1000)
    