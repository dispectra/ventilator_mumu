import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Declare I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# ADS object definition
ads = ADS.ADS1115(i2c)

# Channel definition
chan1 = AnalogIn(ads, ADS.P0, ADS.P1)
chan2 = AnalogIn(ads, ADS.P2)
chan3 = AnalogIn(ads, ADS.P3)

print("{:>5}\t{:>5}".format('raw', 'v'))

while True:
    print("{:>5}\t{:>5.3f}".format(chan1.value, chan1.voltage))
    time.sleep(1)
