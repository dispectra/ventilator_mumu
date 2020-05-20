#!/usr/bin/env python
"""Basic step motor rotation control.

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
import time
from pyA20.gpio import gpio
from pyA20.gpio import port

__author__ = "Samudra Buana"
__copyright__ = "Copyright 2020, SAMx MyID"
__credits__ = ["Samudra Buana"]
__license__ = "GPL"
__version__ = "2.0"
__maintainer__ = __author__
__email__ = "support@osamudra.my.id"

zeroLimSw = port.PA6	#pin 7		(limit switch zero)
maxLimSw = port.PA11	#pin 5		(limit switch max)
clk = port.PA3		#pin 15		(-clock)
cw = port.PA0		#pin 13		(-cw)
en = port.PA1		#pin 11		(-en)
runLed = port.PA12     #pin 3		(LED)
led = port.PA13  #pin 8		(run LED)

gpio.init()
#Driver motor step
gpio.setcfg(clk, gpio.OUTPUT)
gpio.setcfg(cw, gpio.OUTPUT)
gpio.setcfg(en, gpio.OUTPUT)
#Mimit switch
gpio.setcfg(zeroLimSw, gpio.INPUT)
gpio.setcfg(maxLimSw, gpio.INPUT)
#LED
gpio.setcfg(led, gpio.OUTPUT)
gpio.setcfg(runLed, gpio.OUTPUT)

#url = "http://192.168.0.100/ventilator/getParam.php?id=1"	#LAPY
url = "http://localhost/ventilator/getParam.php?id=1"	#Orange-PI
url_mode =  "http://localhost/ventilator/getMode.php?id=1"	#Orange-PI

req = urllib2.Request(url, headers={'Content-type': 'application/json', 'Accept': 'application/json'})
req_mode = urllib2.Request(url_mode, headers={'Content-type': 'application/json', 'Accept': 'application/json'})

try:
    print ("Press CTRL+C to exit")
    
    while True:
		#--- Request for mode settings ---
		#response_mode = urllib2.urlopen(req_mode)
		#the_page = response_mode.read()

		#--- Request for motor settings ---
		response = urllib2.urlopen(req)
		the_page = response.read()
		#convert data to dictionary type; data berikut = type dictionary
		#sehingga bisa akses masing-masing data
		data = json.loads(the_page)
		#print(data) 
		jml_puls = int(data['jml_puls'])
		delay_low = float(data['delay_low'])
		delay_low = delay_low
		delay_high = float(data['delay_high'])
		delay_high = delay_low 
		inspirasi = float(data['inspirasi'])
		espirasi = float(data['espirasi'])
		bpm = int(data['bpm'])
		status_venti = int(data['status_venti'])
		sisa_durasi_espirasi = float(data['sisa_durasi_espirasi'])
		
		#-----testing only-----
		#status_venti=1
		#delay_low = 650
		#delay_high = 650
		#---end testing only---
		if(status_venti==0):
			gpio.output(runLed, 0)
		
		if status_venti==1:
			print ""
			print "Status:"
			print "Ventilator running"
			print data
			print ("jml_puls: ")
			print jml_puls
			print ""
			gpio.output(runLed, 1)
			
		#CW rot; Forward
			gpio.output(en, 1)	
			gpio.output(cw, 0)
			gpio.output(led, 1)
			
			start = time.time()
			
			#delay_low2 = delay_low + 300
			for puls in range(jml_puls): 
				#if puls < 0.5*jml_puls:
					#delay_low2 -=  300/(0.6*jml_puls)
				#else:
					#delay_low2 += 300/(0.5*jml_puls)
				gpio.output(clk, 0)	
				sleep(delay_low /10E5)		# minimum 800 micro sec
				gpio.output(clk, 1)
				sleep(delay_low /10E5)		# minimum 800 micro sec
				#if gpio.input(maxLimSw)==0:
				#	gpio.output(en, 0)
			#delay antara fwd dan rev
			sleep(0.1)	#sleep 100ms
			
        #CCW rot; Reverse
			gpio.output(en,1)
			gpio.output(cw, 1)
			gpio.output(led, 0)
			
			for puls in range(jml_puls+50):
				gpio.output(clk, 0)	
				sleep(delay_low/10E5)		# minimum 800 micro sec
				gpio.output(clk, 1)
				sleep(delay_high/10E5)		# minimum 800 micro sec
				if gpio.input(zeroLimSw)==0:
					gpio.output(en, 0)
					puls = jml_puls
					#print jml_puls
			
        #Stop rot
			gpio.output(en, 0)	
			print ("sisa_durasi_espirasi: ")
			print sisa_durasi_espirasi
			print ""
			#sisa_durasi_espirasi=100
			while(time.time()-start < sisa_durasi_espirasi): 
				sleep(0.000001)
			#sleep(sisa_durasi_espirasi/100000)
			
except KeyboardInterrupt:
	gpio.output(en, 0)
	print ("Selamat tinggal.")
