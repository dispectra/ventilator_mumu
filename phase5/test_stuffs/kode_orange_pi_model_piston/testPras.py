#!/usr/bin/env python
"""Basic dc motor rotation control.

Step Motor Nema 23 3.1Nm (1-unit)
Menggunakan TM6600 shield (1-unit)
TESTED ---OK

1) cw   ---wait 100ms--- 
2) ccw  ---wait 3s--- 
3) stop ---wait 3s--- 
4) kembali ke 1).

Untuk menentukan PA berapa (pin?) atau PB berapa (pin?) lihat gambar pinout: 
	https://www.instructables.com/id/Orange-Pi-One-Python-GPIO-basic/
	
lanjutkan dengan AI untuk camera object detections
	https://medium.com/nanonets/how-to-easily-detect-objects-with-deep-learning-on-raspberrypi-225f29635c74
"""
import os
import sys
import json
import urllib
import urllib2

if not os.getegid() == 0:
    sys.exit('Script must be run as root')

from time import sleep
from pyA20.gpio import gpio
from pyA20.gpio import port

__author__ = "Samudra Buana"
__copyright__ = "Copyright 2020, SAMx MyID"
__credits__ = ["Samudra Buana"]
__license__ = "GPL"
__version__ = "2.0"
__maintainer__ = __author__
__email__ = "support@osamudra.my.id"

zeroLimSw = port.PA6	#pin 7		(M1-)
maxLimSw = port.PA6	#pin 7		(M1-)
clk = port.PA3		#pin 11		(M1+)
cw = port.PA0		#pin 13		(M2-)
en = port.PA1		#pin 15		(M2+)

gpio.init()
#Driver motor step
gpio.setcfg(clk, gpio.OUTPUT)
gpio.setcfg(cw, gpio.OUTPUT)
gpio.setcfg(en, gpio.OUTPUT)
#Mimit switch
gpio.setcfg(zeroLimSw, gpio.INPUT)
gpio.setcfg(maxLimSw, gpio.INPUT)

#url = "http://192.168.0.100/ventilator/getParam.php?id=1"	#LAPY
url = "http://localhost/ventilator/getParam.php?id=1"	#Orange-PI
url_mode =  "http://localhost/ventilator/getMode.php?id=1"	#Orange-PI

req = urllib2.Request(url, headers={'Content-type': 'application/json', 'Accept': 'application/json'})
req_mode = urllib2.Request(url_mode, headers={'Content-type': 'application/json', 'Accept': 'application/json'})



try:
    raw_input("press enter to continue... ")
    gpio.output(en, 0)	
    gpio.output(cw, 0)
    delayq = 500
    i = 0
    while True:
		gpio.output(clk, 0)	
		sleep(delayq/10E5)
		gpio.output(clk, 1)
		sleep(delayq/10E5)
		i += 1
		if i%1000 == 0:
			delayq += 10
			delayq = 3000 if delayq>3000 else delayq
			print(delayq)
		
			
except KeyboardInterrupt:
	gpio.output(en, 1)
	print ("Selamat tinggal.")

	
