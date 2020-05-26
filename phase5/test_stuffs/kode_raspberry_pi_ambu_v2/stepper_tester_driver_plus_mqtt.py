import time
import RPi.GPIO as gpio


# Initialization

## Pin setup/attach
PULSE_PIN = 17
DIR_PIN = 27
ENA_PIN = 22
gpio.setmode(gpio.BCM)
gpio.setup(PULSE_PIN, gpio.OUT)
gpio.setup(DIR_PIN, gpio.OUT)
gpio.setup(ENA_PIN, gpio.OUT)

## Initial value
global ena_now
global dir_now
global pulse_period
global pulse_count_set
ena_now = False
ena_bfr = False
dir_now = False
dir_bfr = False
pulse_period = 2000
pulse_count_set = 1000
pulse_now = False
gpio.output(ENA_PIN, gpio.HIGH)
gpio.output(DIR_PIN, gpio.HIGH)
gpio.output(PULSE_PIN, not pulse_now)

# MQTT client
import paho.mqtt.client as mqtt

BROKER = "localhost"
MQ_PORT = 1883

def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("#")

def on_message(client, userdata, msg):
    global ena_now
    global dir_now
    global pulse_period
    global pulse_count_set
    print(msg.topic+" :"+str(msg.payload))
    if (msg.topic == "root/stepper/ena"):
        ena_now = int(msg.payload)
        print("ena ", ena_now)
    if (msg.topic == "root/stepper/dir"):
        dir_now = int(msg.payload)
        print("dir ", dir_now)
    if (msg.topic == "root/stepper/pulse_period"):
        pulse_period = int(msg.payload)
        print("period ", pulse_period)
    if (msg.topic == "root/stepper/pulse_count_set"):
        pulse_count_set = int(msg.payload)
        print("count ", pulse_count_set)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, MQ_PORT, 60)
#client.user_data_set(ena_now, dir_now, pulse_period, pulse_count_set)
client.loop_start()


# Loop
while True:

## Update ena, dir, pulse_period

    ena_now = True
    dir_now = True
    pulse_period = 2000
    pulse_count_set = 1000
    pulse_count_left = pulse_count_set
    print("ena true, dir true, pulse 2000us", ena_now, dir_now, pulse_period)

    while (pulse_count_left > 0):
        print(ena_now, dir_now, pulse_count_left, pulse_period)
## Required delay for ena & dir transition
        if (ena_now != ena_bfr):
            time.sleep(10/10E5)
            gpio.output(ENA_PIN, not ena_now)
        if (dir_now != dir_bfr):
            time.sleep(10/10E5)
            gpio.output(DIR_PIN, not dir_now)
            
        gpio.output(ENA_PIN, not ena_now)
        gpio.output(DIR_PIN, not dir_now)

## Set pulse
        if (ena_now == True):
            pulse_now = not pulse_now
            gpio.output(PULSE_PIN, not pulse_now)
            time.sleep(pulse_period/10E3)
## Pulse when disabled
        else:
            pulse_now = False
            gpio.output(PULSE_PIN, not pulse_now)
                
        pulse_count_left = pulse_count_left - 1

## Assign value for next loop
        ena_bfr = ena_now
        dir_bfr = dir_now

