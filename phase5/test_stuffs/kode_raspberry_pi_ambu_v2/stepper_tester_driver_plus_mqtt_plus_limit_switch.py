# stepper_driver_v2.1

import time
import RPi.GPIO as gpio


# Initialization

## Stepper driver pin setup/attach
PULSE_PIN = 17
DIR_PIN = 27
ENA_PIN = 22
gpio.setmode(gpio.BCM)
gpio.setup(PULSE_PIN, gpio.OUT)
gpio.setup(DIR_PIN, gpio.OUT)
gpio.setup(ENA_PIN, gpio.OUT)

## Limit switch + button pin
LIM_SWC_MIN = 23
LIM_SWC_MAX = 24
gpio.setup(LIM_SWC_MIN, gpio.IN, pull_up_down=gpio.PUD_UP)
gpio.setup(LIM_SWC_MAX, gpio.IN, pull_up_down=gpio.PUD_UP)
min_lim_flag = not gpio.input(LIM_SWC_MIN)
max_lim_flag = not gpio.input(LIM_SWC_MAX)

## Initial value
global ena_now
global dir_now
global pulse_width
global step_count_set
ena_now = 0
ena_bfr = 0
dir_now = 0
dir_bfr = 0
pulse_width = 1000
step_count_set = 5000
pulse_now = 0
gpio.output(ENA_PIN, not ena_now)
time.sleep(100/10E5)
gpio.output(DIR_PIN, not dir_now)
time.sleep(100/10E5)
gpio.output(PULSE_PIN, not pulse_now)

# MQTT client
import paho.mqtt.client as mqtt

BROKER = "localhost"
MQ_PORT = 1883

def on_connect(client, userdata, flags, rc):
    print("MQTT connected with result code "+str(rc))
    client.subscribe("root/stepper/#")

def on_message(client, userdata, msg):
    global ena_now
    global dir_now
    global pulse_width
    global step_count_set
    print(msg.topic+":"+str(msg.payload))
    if (msg.topic == "root/stepper/ena"):
        ena_now = int(msg.payload)
        print("ena ", ena_now)
    elif (msg.topic == "root/stepper/dir"):
        dir_now = int(msg.payload)
        print(22)
        print("dir ", dir_now)
    elif (msg.topic == "root/stepper/pulse_width"):
        pulse_width = int(msg.payload)
        print("period ", pulse_width)
    elif (msg.topic == "root/stepper/step_count_set"):
        step_count_set = int(msg.payload)
        print("count ", step_count_set)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER, MQ_PORT, 60)
client.loop_start()

# (debug) Logic volt check
#ena_now = 1
#dir_now = 1
#pulse_now = 0
#print("Logic check")
#gpio.output(ENA_PIN, not ena_now)
#gpio.output(DIR_PIN, not dir_now)
#gpio.output(PULSE_PIN, not pulse_now)
#time.sleep(1)

# Loop
while 1:

## Update ena, dir, pulse_width
    step_count_left = step_count_set
#    dir_now = not dir_now

    while (step_count_left > 0):
## Read limit switch
        min_lim_flag = not gpio.input(LIM_SWC_MIN)
        max_lim_flag = not gpio.input(LIM_SWC_MAX)
        if ena_now:
            ena_now = not (min_lim_flag or max_lim_flag)
        #print("Limit switch: ", min_lim_flag, max_lim_flag, ena_now)
## (debug) Step status
        #print(ena_now, dir_now, pulse_width, step_count_left)
## Required delay for ena & dir transition
        if (ena_now != ena_bfr):
            step_count_left = step_count_set
            time.sleep(100/10E5)
            gpio.output(ENA_PIN, not ena_now)
        if (dir_now != dir_bfr):
            time.sleep(100/10E5)
            gpio.output(DIR_PIN, not dir_now)
## Set enable and direction
        gpio.output(ENA_PIN, not ena_now)
        gpio.output(DIR_PIN, not dir_now)
## Set pulse
        if (ena_now == 1):
            pulse_now = not pulse_now
            gpio.output(PULSE_PIN, not pulse_now)
            time.sleep(pulse_width/10E5)
            step_count_left = step_count_left - 1
## Pulse when disabled
        else:
            pulse_now = 0
            gpio.output(PULSE_PIN, not pulse_now)
## Assign value for next step
        ena_bfr = ena_now
        dir_bfr = dir_now
#        pulse_width_bfr = pulse_width_now

