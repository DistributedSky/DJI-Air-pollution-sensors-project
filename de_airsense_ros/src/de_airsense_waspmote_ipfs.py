#! /usr/bin/python

import serial
import time
import array
import os
import errno
import ipfsapi

DEFAULT_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
INTERVAL = 1

def main():
    frame = ""
    frame_array = []
    fileName = "data_"

    serial_port = serial.Serial(DEFAULT_PORT, BAUDRATE, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

    while True:
        try:    
            while serial_port.in_waiting > 0:
                byte = serial_port.read()

                if byte == b"\n":
                    print (frame)
                    frame += byte.decode()
                    frame_array.append(frame)
                    frame = ""
                    continue

                if byte != b"\x86" and byte != b"\x00":
                    frame += byte.decode()

           
            time.sleep(INTERVAL)

        except KeyboardInterrupt: 
            dir = os.path.dirname(__file__)
            folderName = os.path.join(dir, 'sensor_data/')
            if not os.path.exists(os.path.dirname(folderName)):
                try:
                    os.makedirs(os.path.dirname(folderName))
                except OSError as exc: # Guard against race condition
                    if exc.errno != errno.EEXIST:
                        raise
                        
            timestr = time.strftime("%Y%m%d_%H%M%S_")
            fileName = folderName + fileName + timestr + ".txt"
            with open (fileName, "w") as f:
                for item in frame_array:
                    f.write(item)

            api = ipfsapi.connect('127.0.0.1', 5001)
            res = api.add(fileName)
            print (res)
            if res != None:
                os.rename(fileName, fileName[:-4] + res["Hash"] + fileName[-4:])
            print ("\nExit")
            break

if __name__ == "__main__":
    
    main()