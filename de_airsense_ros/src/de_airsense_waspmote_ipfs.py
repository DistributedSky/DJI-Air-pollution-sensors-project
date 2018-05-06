#! /usr/bin/python

import serial
import time
import array
import os
import errno
import ipfsapi
from std_msgs.msg import UInt8
from sensor_msgs.msg import NavSatFix
import rospy

DEFAULT_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
RATE_HZ = 1
IN_AIR_STANDBY = 3
ON_GROUND = 1

status_in_air = False
altitude = 0
frame_array = []

def write_send_data():
    global frame_array
    fileName = 'data_'
    dir = os.path.dirname(__file__)
    folderName = os.path.join(dir, 'sensor_data/')
    if not os.path.exists(os.path.dirname(folderName)):
        try:
            os.makedirs(os.path.dirname(folderName))
        except OSError as exc: # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
                
    timestr = time.strftime('%Y%m%d_%H%M%S_')
    fileName = folderName + fileName + timestr + '.txt'
    with open (fileName, 'w') as f:
        for string in frame_array:
            f.write(string)
    frame_array = []

    api = ipfsapi.connect('127.0.0.1', 5001)
    # api = ipfsapi.connect('52.178.98.62', 9095)
    res = api.add(fileName)
    print (res)
    time.sleep(1)
    if res != None:
        os.rename(fileName, fileName[:-4] + res['Hash'] + fileName[-4:])

def status_cb(data): 
    global status_in_air
    if data.data == IN_AIR_STANDBY and status_in_air == False:
        status_in_air = True
        print 'Start to write data'
    elif data.data == ON_GROUND and status_in_air == True:
        print 'Stop to write data'
        write_send_data()
        status_in_air = False

def position_cb(data):
    global altitude
    altitude = data.altitude

if __name__ == '__main__':
    print 'Starting waspmote gas sensors...'
    rospy.init_node('de_airsense_waspmore_ipfs')
    rospy.Subscriber('dji_sdk/flight_status', UInt8, status_cb)
    rospy.Subscriber('dji_sdk/gps_position', NavSatFix, position_cb)

    rate = rospy.Rate(RATE_HZ)
    frame = ''
    serial_port = serial.Serial(DEFAULT_PORT, BAUDRATE, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
    waspmote_ready = False
    while not rospy.is_shutdown():
        try:    
            while serial_port.in_waiting > 0:
                if waspmote_ready == False:
                    waspmote_ready = True
                    print 'Waspmote gas sensors is ready'
                byte = serial_port.read()

                if status_in_air:
                    if byte == b'\n':
                        frame += byte.decode()  
                        frame += 'ALT:{0:.1f}#{1:s}'.format(altitude, time.strftime('%Y/%m/%d %H:%M:%S'))
                        frame_array.append(frame)
                        print (frame)
                        frame = ''
                        continue

                    if byte != b'\x86' and byte != b'\x00':
                        frame += byte.decode()
            rate.sleep()

        except KeyboardInterrupt: 
            print '\nExit'
            break

