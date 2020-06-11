import time
import Adafruit_ADS1x15
import paho.mqtt.client as mqtt

def delayMicroseconds(dt):
    dt = dt/10E5
    now = time.perf_counter()
    while (time.perf_counter() - now < dt):
        pass

def on_connect(client, userdata, flags, rc):
    #print("M> C "+str(rc))
    pass
    
def on_publish(client, userdata, result):
    #print("M> P")
    pass
    
client = mqtt.Client()
client.on_connect = on_connect
client.on_publish = on_publish
BROKER = "localhost"
PORT = 1883
client.connect(BROKER, PORT)
client.loop_start()


adc = Adafruit_ADS1x15.ADS1115()
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

#  - 2/3 = +/-6.144V#  -   1 = +/-4.096V#  -   2 = +/-2.048V
#  -   4 = +/-1.024V#  -   8 = +/-0.512V#  -  16 = +/-0.256V
GAIN1 = 16
GAIN2 = 2/3
GAIN3 = 2/3
raw1 = 0
raw2 = 0
raw3 = 0

#print("1 \t 2 \t 3")
#print('-' * 20)

#while True:
#    raw1 = adc.read_adc_difference(0, gain=GAIN1)
#    raw2 = adc.read_adc(2, gain=GAIN2)
#    raw3 = adc.read_adc(3, gain=GAIN3)
    
#    print(raw1, "\t", raw2, "\t", raw3)
#    client.publish("raw_pressure_val", raw3)
#    delayMicroseconds(20E3)

def samplingRoutine():
    try:
        while True:
            raw1 = adc.read_adc_difference(0, gain=GAIN1)
            raw2 = adc.read_adc(2, gain=GAIN2)
            raw3 = adc.read_adc(3, gain=GAIN3)
            
            #print(raw1, "\t", raw2, "\t", raw3)
            client.publish("root/adc/oxy_raw", raw1)
            client.publish("root/adc/flow_raw", raw2)
            client.publish("root/adc/pres_raw", raw3)
            delayMicroseconds(20E3)

    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        exit()
        
    except:
        delayMicroseconds(10)
        samplingRoutine()


samplingRoutine()
