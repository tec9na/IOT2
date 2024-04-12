from machine import Pin, ADC, SoftI2C, SPI
import time
import dht
from micropython_rfm9x import *
from adc_sub import ADC_substitute


#motor og LED
pumpe_pin = 16 # sætter pumpe pin til 16
pumpe = Pin(pumpe_pin, Pin.OUT)        # opretter et pinobjekt til pumpen
led_pin = 17                           # vi gør det samme med vores LED
led = Pin(led_pin, Pin.OUT)            # opretter et pin objekt til led

#soil sensor
soil = ADC(Pin(32))                    # opretter pin objekt til soil sensor (fra ADC Klassen, fra machine modulet)
m = 100                                 # max moisture

min_moisture = 0                        # lavest adc værdi  0 = 0 %
max_moisture = 4095                     # højest adc værdi 4095 = 100%

soil.atten(ADC.ATTN_11DB)                #full range: 3.3v
soil.width(ADC.WIDTH_12BIT)              #range 0 to 4095

#DHT 11
sensor_id = "DHT11"                    # The sensor ID
pin_dht11 = 26                         # The ultra sound digital trigger GPIO output pin

sensor = dht.DHT11(Pin(pin_dht11))    # laver et pin objekt som vi kalder sensor, gennem DHT klassen
#Battery
pin_battery = 35                       # The battery measurement input pin, ADC1_6

battery = ADC_substitute(pin_battery)  # The battery object
ip1 = 2038                             # Insert own measured value here when the battery is fully discharged
ip2 = 2921                             # Insert own measured value here when the battery is fully charged
bp1 = 0                                # Battery percentage when fully discharged
bp2 = 100                              # Battery percentage when fully charged
alpha = (bp2 - bp1) / (ip2 - ip1)      # alpha is the slope in the first degree equation bp = alpha * input + beta
beta = bp1 - alpha * ip1               # beta is the crossing point on the y axis
# Previous values Batteri
prev_bat_pct = -1     

# Pumpe
threshold_value = 20                 # Threshold value for turning on the pump
next_time = 0
pumpe_on_time= 5                    # tænd pumpe i X sekunder
pumpe_pause = 5
pumpe_is_on = 0                        # 0: pumpe off, 1: pumpe on, 2: sive tid
vand_sive_tid = 60                      #hvor lang tid skal vand sive
vand_sive_tid_next = 0
pumpe_stop_time = 0
#Lora
lora_next_time = 0
lora_interval = 5                  # send lora Data hver X sekund
###########################################################################################################################3
#ESP32 Lora modul initialisering
RESET = Pin(14, Pin.OUT)
#spi = SPI(2, baudrate=1000000, polarity=0, phase=0, bits=8, firstbit=0, sck=Pin(5), mosi=Pin(18), miso=Pin(19))
CS = Pin(5, Pin.OUT)
#RESET = Pin(22, Pin.OUT)
spi = SPI(2, baudrate=10000, polarity=0, phase=0, bits=8, firstbit=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))


RADIO_FREQ_MHZ = 858.0
# Initialze RFM radio
rfm9x = RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

# Note that the radio is configured in LoRa mode so you can't control sync
# word, encryption, frequency deviation, or other settings!

# You can however adjust the transmit power (in dB).  The default is 13 dB but
# high power radios like the RFM95 can go up to 23 dB:
rfm9x.tx_power = 23
# time.sleep(1)  # 
################################## Funktioner
def get_battery_percentage():          # The battery voltage percentage
    ip = battery.read_adc()            # Either use this or the one below. Remove if not used
    
    bp = alpha * ip + beta             # Calculate the battery percentage based on the measured input value
    bp = int(bp)                       # lav bp om til en integer
    
    # Make sure noise on the readings don't result in invalid values
    if bp < 0:                         
        bp = 0
    elif bp > 100:
        bp = 100
    
    return bp

def moisture():                     # funktion til at få Moisture i procent værdi
    soil.read()
    m = (max_moisture-soil.read())*100/(max_moisture-min_moisture)
    return m

def data():                      #funktion at samle alt vores data og lægge det i en f string
    soil.read()
    sensor.measure()
    m = (max_moisture-soil.read())*100/(max_moisture-min_moisture)
    moisture = '{:.1f} %'.format(m)     
    sensor_value_temp = sensor.temperature()
    sensor_value_hum = sensor.humidity()
    bat_pct = get_battery_percentage()
    values = [sensor_value_temp, sensor_value_hum, m, bat_pct]
    m = int(m + 0.5)
    return f"{sensor_value_temp},{sensor_value_hum},{m},{bat_pct}"   #vi bruger en f string til at return vores variabler(vores værdier) som en string

packet = data()  # vi laver en variabel kaldt packet, som er = vores data funktion
data = f"{packet}" # vi laver en variabel kaldt data, som er = vores data funktion forklædt packet
led.on()
################################# Main
# Send a packet.  Note you can only send a packet up to 252 bytes in length.
# This is a limitation of the radio packet size, so if you need to send larger
# amounts of data you will need to break it into smaller send calls.  Each send
# call will wait for the previous one to finish before continuing.
while True:
    ##### Lora Sender time.ticks_diff() skal bruge to argumenter
    if time.ticks_diff(time.ticks_ms(), lora_next_time) > 0 and pumpe_is_on != 1: #to parametre skal opfyldes før lora sender
        rfm9x = RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)  # bliver nød til at reinitialiseret fordi pumpe laver for meget støj
        rfm9x.send(bytes(data, "utf-8")) #encodes i utf-8 
        lora_next_time = time.ticks_ms() + lora_interval* 1000
    #pumpe control
    # Check if the sensor value exceeds the threshold
    temp = sensor.temperature()
    hum = sensor.humidity()
    if m < threshold_value and pumpe_is_on == 0 and temp >= 1 and hum <= 80:   #pumpe kører i X tid, hvis moisture % er over valgt threshold_value, og pumpe ikke er tændt, og temperatur er 1 grader eller over, og humidity er 80 grader eller mindre
        # Turn on the pump
        pumpe.value(1)
        pumpe_stop_time = time.ticks_ms() + pumpe_on_time*1000
        pumpe_is_on = 1
    if pumpe_is_on == 1 and time.ticks_diff(time.ticks_ms(), pumpe_stop_time) > 0: # pumpe stopper når x tid er gået
        # Turn off the pump
        pumpe_is_on = 2
        vand_sive_tid_next = time.ticks_ms() + vand_sive_tid*1000 
        pumpe.value(0)
    if time.ticks_diff(time.ticks_ms(), vand_sive_tid_next) > 0 and pumpe_is_on == 2: #lad vand sive ind i X tid
        # Turn off the pump
        pumpe_is_on = 0
        pumpe.value(0)

