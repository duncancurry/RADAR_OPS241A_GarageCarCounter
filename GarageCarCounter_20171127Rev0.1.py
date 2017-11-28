#!/usr/bin/env python3
####################################################
#
# Description: Parking garage car counter using
#              OmniPreSense OPS241-A RADAR Sensor
# By: Duncan Curry
#
# Last Modified: 2017/11/27
#
# Rev: 0.1 :    0. 2017/11/27
#               1. Initial release
# 
#####################################################
# Modifiable parameters
init_car_count = 0 # Initial number of cars in garage
Ops241A_object_max_speed = 20 # max speed in mph that a car can travel in order to be tracked; anything faster is not a car
Ops241A_object_min_speed = 4 # min speed in mph that a car can travel in order to be tracked; anything slower is not a car
idle_speed_wait_time = 3.0  # time in h,m,s waiting to receive an initial car speed data from OPS241A sensor before printing heartbeat message
tracking_speed_wait_time = 2.0 # time in h,m,s waiting for next car speed data from OPS241 sensor when already tracking a car
min_car_tracking_time = 1.0 # min time in h,m,s that car needs to be tracked for it to be counted
wifi_wait_time = 1.0 # time in secs to wait for wifi to connect for MQTT Cayenne

# Ops241A module setting
Ops241A_Speed_Output_Units = 'US'  # mph
Ops241A_Direction_Control = 'OD'   # direction ON
Ops241A_Sampling_Frequency = 'S2'  # 20Ksps
Ops241A_Transmit_Power = 'P7'      # max power
Ops241A_Threshold_Control = 'QX'   # squelch -100
Ops241A_Data_Accuracy = 'F5'       # 5 decimal reporting
Ops241A_Debug_mode1 = 'Dr'         # Turn Red LED off
Ops241A_Debug_mode2 = 'Dy'         # Turn Yellow LED off
Ops241A_RGB_LED_Lighting_Control = '^0'   # Turn all LEDs off
Ops241A_Module_Information = '??'

#MQTT settings
username = "4a8b2810-3812-11e7-8d6e-f398b869a12a"
password = "212095c5b81b95162cd069dc99ad0f1e5df2c537"
clientid = "47de3df0-d349-11e7-a824-91d417d3812a"

#####################################################
# Do not make any changes below here
#####################################################
# Import time, decimal, serial, reg expr, sys
import os
import sys
import time
import datetime
# from time import *
from decimal import *
import serial
import re
import paho.mqtt.client as mqtt


# Initialize the USB port to read from the OPS-241A module
ser=serial.Serial(
    port = '/dev/ttyACM0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 1,
    writeTimeout = 2
)
ser.flushInput()
ser.flushOutput()

# sendSerialCommand: function for sending commands to the OPS-241A module
def sendSerCmd(descrStr, commandStr, VerifyBool) :
    data_for_send_str = commandStr
    data_for_send_bytes = str.encode(data_for_send_str)
    print(descrStr, commandStr)
    ser.write(data_for_send_bytes)
    # Initialize message verify
    ser_message_start = '{'
    ser_write_verify = VerifyBool
    # Print out module response to command string if VerifyBool is True
    while ser_write_verify :
        data_rx_bytes = ser.readline()
        data_rx_length = len(data_rx_bytes)
        if (data_rx_length != 0) :
            data_rx_str = str(data_rx_bytes)
            if data_rx_str.find(ser_message_start) :
                print(data_rx_str)
                ser_write_verify = False
            
# Initialize Ops241A Module and verify if True
print("\nInitializing Ops241A Module")
sendSerCmd("\nSet Speed Output Units: ", Ops241A_Speed_Output_Units, True)
sendSerCmd("\nSet Direction Control: ", Ops241A_Direction_Control, True)
sendSerCmd("\nSet Sampling Frequency: ", Ops241A_Sampling_Frequency, True)
sendSerCmd("\nSet Transmit Power: ", Ops241A_Transmit_Power, True)
sendSerCmd("\nSet Threshold Control: ", Ops241A_Threshold_Control, True)
sendSerCmd("\nSet Data Accuracy: ", Ops241A_Data_Accuracy, True)
sendSerCmd("\nSet Red LED Control: ", Ops241A_Debug_mode1, False)
sendSerCmd("\nSet Yellow LED Control: ", Ops241A_Debug_mode2, False)
sendSerCmd("\nSet RGB LED Lighting Control: ", Ops241A_RGB_LED_Lighting_Control, False)
sendSerCmd("\nModule Information: ", Ops241A_Module_Information, True)

ser=serial.Serial(
    port = '/dev/ttyACM0',
    baudrate = 9600,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 0.01,
    writeTimeout = 2
    )

# getSpeed: Function to check and return valid speed data from OPS241A
#           Positive speed -> object approaching sensor
#           Negative speed -> object moving away from sensor
def getValidSpeed() :
    validSpeed_boolean = False
    objectSpeed_float = 0.0
    Ops241_rx_bytes = ser.readline()
    Ops241_rx_bytes_length = len(Ops241_rx_bytes)
    if (Ops241_rx_bytes_length != 0) :
        Ops241_rx_str = str(Ops241_rx_bytes)
        if Ops241_rx_str.find('{') == -1 :
            # Speed data found
            objectSpeed_float = float(Ops241_rx_bytes)
            # Check if speed is valid for object
            objectSpeed_abs_float = abs(objectSpeed_float)
            if objectSpeed_abs_float > Ops241A_object_min_speed and objectSpeed_abs_float < Ops241A_object_max_speed :
                validSpeed_boolean = True
    return validSpeed_boolean, objectSpeed_float

# wait for wireless connection after boot
print('\nConnecting to wifi, please wait 30s...\n')
time.sleep(wifi_wait_time) # Sleep to allow wireless to connect before starting MQTT

# connect to MQTT
# print('Connecting to MQTT, please wait...')
# mqttc = mqtt.Client(client_id=clientid)
# mqttc.username_pw_set(username, password=password)
# mqttc.connect("mqtt.mydevices.com", port=1883, keepalive=60)
# mqttc.loop_start()
# topic_ops241A_speed = "v1/" + username + "/things/" + clientid + "/data/1"    

# Initialize
car_count = init_car_count
done = False
while not done :
    # Flush serial buffers
    ser.flushInput()
    ser.flushOutput()
    #
    # Car counting and tracking loops
    tracking = False
    while not tracking :
        # Reset valid tracking
        valid_tracking = False
        # Wait for valid speed
        # Initialize wait timer        
        idle_start_time = time.time()
        idle_current_time = idle_start_time
        valid_speed = False
        while not valid_speed :
            # Get speed from Ops241A   
            valid_speed, speed_float = getValidSpeed()
            if valid_speed == False :
                idle_current_time = time.time()
                idle_delta_time = idle_current_time - idle_start_time
                if idle_delta_time > idle_speed_wait_time :
                    print('No car motion detected, waiting ... Car count = ', car_count)
                    # Send car count to Cayenne
#                    try:
#                        if car_count is not None :
#                            car_count_str = "Car count at Mountain View Library Garage = " + str(car_count)
#                            mqttc.publish(topic_ops241A_speed, payload=car_count_str, retain=True)
#                    except (EOFError, SystemExit, KeyboardInterrupt) :
#                        mqttc.disconnect()
                    # Reset wait timer
                    idle_start_time = idle_current_time
        # Valid speed received
        newSpeed = speed_float
        # Begin tracking
        tracking = True
    while tracking :
        # Initialize tracking timer
        tracking_start_time = time.time()
        tracking_current_time = tracking_start_time
        tracking_delta_time = 0.0
        same_direction = True
        while not valid_tracking :
            wait_start_time = time.time()
            wait_current_time = wait_start_time
            wait_delta_time = 0.0
            valid_speed = False
            while same_direction :
                # Get speed from Ops241A
                valid_speed, speed_float = getValidSpeed()
                if valid_speed == False :
                    # Wait for valid speed data
                    wait_current_time = time.time()
                    wait_delta_time = wait_current_time - wait_start_time
                    # Check if wait timer has timed out
                    if wait_delta_time > tracking_speed_wait_time :
                        # Check if tracking was valid
                        if not valid_tracking :
                            print('Minimum tracking time failure, tracking lost, resetting')
                        else:
                            # Tracking was already valid and car went out of range
                            # Inc/dec car count based upon positive or negative current speed
                            if newSpeed > 0 :
                                # Car entered garage, increment count
                                car_count += 1
                                print('Car entered garage, new car count = ', car_count)
                            elif newSpeed < 0 :
                                # Car exited garage, decrement count with floor of 0
                                car_count -= 1
                                print('Car exited garage, new car count = ', car_count)
                                if car_count < 0 :
                                    car_count = 0
                                    print('Whoops, did not realize there were any cars still in the garage, car count is now 0')
                            else :
                                # Should never get 0 speed
                                print('Whoops, car speed was 0! No change to car count, car count = ', car_count)
                        # Set flags to exit all tracking loops
                        same_direction = False
                        valid_tracking = True
                        tracking = False
                # Valid speed received
                else :
                    # Save old and new speeds                      
                    oldSpeed = newSpeed
                    newSpeed = speed_float
                    # print('DEBUG: oldSpeed = ', oldSpeed, 'newSpeed = ', newSpeed)
                    # Test if directions are the same
                    if oldSpeed * newSpeed > 0 :
                        # Direction is the same
                        same_direction = True
                        tracking_current_time = time.time()
                        tracking_delta_time = tracking_current_time - tracking_start_time
                        # Check if tracking time is long enough to be valid
                        if tracking_delta_time > min_car_tracking_time :
                            valid_tracking = True
                            # Continue tracking and getting speeds until direction change or wait timeout
                        # Reset wait timer
                        wait_start_time = time.time()
                    else :
                        # Direction changed
                        same_direction = False
                        if valid_tracking == True :
                            # Tracking was valid and car is going past, or new car is coming opposite direction
                            # Inc/dec car count based upon positive or negative previous speed
                            if oldSpeed > 0 :
                                # Car entered garage, increment count
                                car_count += 1
                                print('Car entered garage, new car count = ', car_count)
                            elif oldSpeed < 0 :
                                # Car exited garage, decrement count with floor of 0
                                car_count -= 1
                                print('Car exited garage, new car count = ', car_count)
                                if car_count < 0 :
                                    car_count = 0
                                    print('Whoops, did not realize there were any cars still in the garage, car count is now 0')
                            else :
                                # Should never get 0 speed
                                print('Whoops, car speed was 0! No change to car count, car count = ', car_count)
                        else :
                             # Tracking time too short and not valid
                             print('Direction changed before tracking lock, restart tracking')
                        # Reset valid tracking and contine to track new object   
                        valid_tracking = False
                        # Reset tracking timer
                        same_direction = True
                        tracking_start_time = time.time()
                            
                                          
                            
 