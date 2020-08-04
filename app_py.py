#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from snipsTools import SnipsConfigParser
from hermes_python.hermes import Hermes
from hermes_python.ontology import *
import io

CONFIG_INI = "config.ini"


from matrix_lite import led,gpio
from time import sleep
from math import pi, sin


##MODBUS 
import minimalmodbus

instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 1, mode = 'rtu') # port name, slave address - Use multiple objects to talk with multiple power meters
instrument.serial.baudrate = 9600
from threading import Thread


import serial
import time
ser = serial.Serial()
ser.port='/dev/ttyUSB2'
ser.baudrate=115200
ser.open()
ser.write("\r\n\r\n".encode()) # Hit enter a few times to wake the printer
time.sleep(2)   # Wait for printer to initialize
ser.flushInput()
##3D Printer



relayPin = 0
gpio.setFunction(relayPin, 'DIGITAL')
gpio.setMode(relayPin, "output")
gpio.setDigital(relayPin,1)


MQTT_IP_ADDR = "localhost"
MQTT_PORT = 1883
MQTT_ADDR = "{}:{}".format(MQTT_IP_ADDR, str(MQTT_PORT))

availableDevice = ['printer']

class snips_power_app(object):
    def __init__(self):
        # get the configuration if needed
        try:
            self.config = SnipsConfigParser.read_configuration_file(CONFIG_INI)
        except :
            self.config = None

        # start listening to MQTT
        self.start_blocking()
        
    # --> powerMonitor callback functions

    def getVolts_callback(self, hermes, intent_message):
        hermes.publish_end_session(intent_message.session_id, "")
        print ('[Received] intent: {}'.format(intent_message.intent.intent_name))

        device = None
        if intent_message.slots:
            device = intent_message.slots.deviceName.first().value
            if device not in availableDevice:
                device = None

        if device is None:
            reply = "No device specified"
        else:
            reply = "Voltage received by "+str(device) +" is " + str(self.get_volts()) + " Volts"

        hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
        hermes.publish_end_session(intent_message.session_id, reply)

    def getCurrent_callback(self, hermes, intent_message):
        hermes.publish_end_session(intent_message.session_id, "")
        
        print ('[Received] intent: {}'.format(intent_message.intent.intent_name))

        device = None
        if intent_message.slots:
            device = intent_message.slots.deviceName.first().value
            if device not in availableDevice:
                device = None

        if device is None:
            reply = "No device specified"
        else:
            reply = "Current consumed by "+str(device) +" is " + str(self.get_current()) + " Amps"

        hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
        hermes.publish_end_session(intent_message.session_id, reply)

    def getPower_callback(self, hermes, intent_message):
        hermes.publish_end_session(intent_message.session_id, "")
        
        print ('[Received] intent: {}'.format(intent_message.intent.intent_name))

        device = None
        if intent_message.slots:
            device = intent_message.slots.deviceName.first().value
            if device not in availableDevice:
                device = None

        if device is None:
            reply = "No device specified"
        else:
            reply = "Power consumed by "+str(device) +" is " + str(self.get_power()) + " Watts"

        hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
        hermes.publish_end_session(intent_message.session_id, reply)

    def getEnergy_callback(self, hermes, intent_message):
        hermes.publish_end_session(intent_message.session_id, "")
        
        print ('[Received] intent: {}'.format(intent_message.intent.intent_name))

        device = None
        if intent_message.slots:
            device = intent_message.slots.deviceName.first().value

            if device not in availableDevice:
                device = None

        if device is None:
            reply = "No device specified"
        else:
            reply = "Total Energy consumed by "+str(device) +" is " + str(self.get_energy()) + " watt hours"

        hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
        hermes.publish_end_session(intent_message.session_id, reply)

    def turnOn_callback(self, hermes, intent_message):
        hermes.publish_end_session(intent_message.session_id, "")
        
        print ('[Received] intent: {}'.format(intent_message.intent.intent_name))

        device = None
        if intent_message.slots:
            device = intent_message.slots.deviceName.first().value
            if device not in availableDevice:
                device = None

        if device is None:
            reply = "No device specified"
        else:
            reply = "Turning "+str(device) + " on"
            gpio.setDigital(relayPin,0)
            
        

        hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
        hermes.publish_end_session(intent_message.session_id, reply)

    def turnOff_callback(self, hermes, intent_message):

        hermes.publish_end_session(intent_message.session_id, "")
        print ('[Received] intent: {}'.format(intent_message.intent.intent_name))

        device = None
        if intent_message.slots:
            device = intent_message.slots.deviceName.first().value
            # check if valid
            if device not in availableDevice:
                device = None

        if device is None:
            reply = "No device specified"
        else:
            reply = "Turning "+str(device) + " off"
            gpio.setDigital(relayPin,1)
        hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
        hermes.publish_end_session(intent_message.session_id, reply)

    def setMaxCurrent_callback(self, hermes, intent_message):
        hermes.publish_end_session(intent_message.session_id, "")
        print ('[Received] intent: {}'.format(intent_message.intent.intent_name))
        device = None
        maxCurrent = None
        if intent_message.slots:
            device = intent_message.slots.deviceName.first().value
            # check if valid
            if device not in availableDevice:
                device = None
            maxCurrent = intent_message.slots.maxCurrent.first().value

        if device is None:
            reply = "No device specified"
            hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
            hermes.publish_end_session(intent_message.session_id, reply)
        elif maxCurrent is None:
            reply = "No current specified"
            hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
            hermes.publish_end_session(intent_message.session_id, reply)
        else:
            t1 = Thread(target=self.monitorCurrent, args=(intent_message,hermes,device,maxCurrent))
            t1.start()

    # --> 3D printer control callback functions

    def setTemp_callback(self, hermes, intent_message):

        hermes.publish_end_session(intent_message.session_id, "")
        print ('[Received] intent: {}'.format(intent_message.intent.intent_name))

        endEffector = None
        temp = None
        if intent_message.slots:
             endEffector = intent_message.slots.bedOrHotend.first().value
             temp = intent_message.slots.temperature.first().value
        
        if endEffector is None:
            reply = "End effector not specified"
            hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
            hermes.publish_end_session(intent_message.session_id, reply)

        elif temp is None:
            reply = "Temperature not specified"
            hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
            hermes.publish_end_session(intent_message.session_id, reply)

        else:
            if endEffector == 'bed':
                if int(temp) > 90:
                    reply = "Temp too high for bed. Maximum is 90 degrees celsius"
                    hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
                    hermes.publish_end_session(intent_message.session_id, reply)
                else:
                    reply = "Setting "+str(endEffector) + " to" + str(temp) + " degrees celsius"
                    command = 'M140 S' +str(temp) +'\n'
                    self.printserial(command)
                    hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
                    hermes.publish_end_session(intent_message.session_id, reply)

            elif endEffector == 'hotend':
                if int(temp) > 220:
                    reply = "Temp too high for hotend. Maximum is 220 degrees celsius"
                    hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
                    hermes.publish_end_session(intent_message.session_id, reply)
                else:
                    reply = "Setting "+str(endEffector) + " to" + str(temp) + " degrees celsius"
                    command = 'M104 S' +str(temp) +'\n'
                    self.printserial(command)
                    hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
                    hermes.publish_end_session(intent_message.session_id, reply)

    def goHome_callback(self, hermes, intent_message):
        hermes.publish_end_session(intent_message.session_id, "")
        
        reply = "Homing all axes"
        command = 'G28\n'
        self.printserial(command)
        hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
        hermes.publish_end_session(intent_message.session_id, reply)


    def move_callback(self, hermes, intent_message):

        hermes.publish_end_session(intent_message.session_id, "")
        print ('[Received] intent: {}'.format(intent_message.intent.intent_name))


        axis = None
        value = None

        if intent_message.slots:
             axis = intent_message.slots.axis.first().value
             value = intent_message.slots.coordinate.first().value

        if axis is None:
            reply = "No axis is specified"
            hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
            hermes.publish_end_session(intent_message.session_id, reply)
        if value is None:
            reply = "No temp is specified"
            hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
            hermes.publish_end_session(intent_message.session_id, reply)
        else:
            reply = "Moving"
            command = 'G1 '+str(axis)+str(value)+'\n'
            self.printserial(command)
            hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
            hermes.publish_end_session(intent_message.session_id, reply)
                    
    
        

    # --> Master callback function, triggered everytime an intent is recognized
    def master_intent_callback(self,hermes, intent_message):
        self.showlights()
        coming_intent = intent_message.intent.intent_name

        #========================== Power Monitor Functions ========================= #
        if coming_intent == 'yasaspeiris:getVoltage':  ## Replace yasaspeiris with your username for snips console
            self.getVolts_callback(hermes, intent_message)
        elif coming_intent == 'yasaspeiris:turnOn':
            self.turnOn_callback(hermes, intent_message)
        elif coming_intent == 'yasaspeiris:turnOff':
            self.turnOff_callback(hermes, intent_message)
        elif coming_intent == 'yasaspeiris:getCurrent':
            self.getCurrent_callback(hermes, intent_message)
        elif coming_intent == 'yasaspeiris:getPower':
            self.getPower_callback(hermes, intent_message)
        elif coming_intent == 'yasaspeiris:getEnergy':
            self.getEnergy_callback(hermes, intent_message)
        elif coming_intent == 'yasaspeiris:setMaxCurrent':
            self.setMaxCurrent_callback(hermes, intent_message)

        #========================== 3D Printer Controls ========================= #
        elif coming_intent == 'yasaspeiris:setTemp':
            self.setTemp_callback(hermes, intent_message)
        elif coming_intent == 'yasaspeiris:goHome':
            self.goHome_callback(hermes, intent_message)
        elif coming_intent == 'yasaspeiris:move':
            self.move_callback(hermes, intent_message)


        # more callback and if condition goes here...

    # --> Register callback function and start MQTT
    def start_blocking(self):
        with Hermes(MQTT_ADDR) as h:
            h.subscribe_intents(self.master_intent_callback).start()


    def get_volts(self):
        try:
            voltage = instrument.read_register(0x0000, 1, functioncode=4, signed=False)
            return voltage
        except ValueError:
            return self.get_voltage()
        except IOError:
            return 0

    def get_current(self):
        try:
            current_low = instrument.read_register(0x0001, 0, functioncode=4, signed=False)
            current_high = instrument.read_register(0x0002, 0, functioncode=4, signed=False)
            current = (current_high << 8 | current_low)/1000.0
            return current
        except ValueError:
            return self.get_current()
        except IOError:
            return 0

    def get_power(self):
        try:
            power_low = instrument.read_register(0x0003, 0, functioncode=4, signed=False)
            power_high = instrument.read_register(0x0004, 0, functioncode=4, signed=False)
            power = (power_high << 8 | power_low)/10.0
            return power
        except ValueError:
            return self.get_power()
        except IOError:
            return 0

    def get_energy(self):
        try:
            energy_low = instrument.read_register(0x0005, 0, functioncode=4, signed=False)
            energy_high = instrument.read_register(0x0006, 0, functioncode=4, signed=False)
            energy = (energy_high << 8 | energy_low)
            return energy
        except ValueError:
            return self.get_energy()
        except IOError:
            return 0
        
    def monitorCurrent(self,intent_message,hermes,device,maxCurrent):
        while True:
            try:
                current_low = instrument.read_register(0x0001, 0, functioncode=4, signed=False)
                current_high = instrument.read_register(0x0002, 0, functioncode=4, signed=False)
                current = (current_high << 8 | current_low)
                if current > maxCurrent :
                    reply = "Max Current Reached. Turning "+str(device) + " off"
                    gpio.setDigital(relayPin,1)
                    hermes.publish_start_session_notification(intent_message.site_id, reply,"snips_power_app")
                    hermes.publish_end_session(intent_message.session_id, reply)
                    break


            except ValueError:
                pass
            except IOError:
                pass

    def showlights(self):
        everloop = ['black'] * led.length
        ledAdjust = 1.01 # MATRIX Voice
        frequency = 0.375
        counter = 0.0
        tick = len(everloop) - 1
        for m in range(0,36):
            # Create rainbow
            for i in range(len(everloop)):
                r = round(max(0, (sin(frequency*counter+(pi/180*240))*155+100)/10))
                g = round(max(0, (sin(frequency*counter+(pi/180*120))*155+100)/10))
                b = round(max(0, (sin(frequency*counter)*155+100)/10))

                counter += ledAdjust

                everloop[i] = {'r':r, 'g':g, 'b':b}

            # Slowly show rainbow
            if tick != 0:
                for i in reversed(range(tick)):
                    everloop[i] = {}
                tick -= 1

            led.set(everloop)

            sleep(.035)
        everloop = ['black'] * led.length
        led.set(everloop)

    def printserial(self,command):

        ser.write (command.encode())
        out = ser.readline()
        print (out)

        


    

if __name__ == "__main__":
    snips_power_app()
