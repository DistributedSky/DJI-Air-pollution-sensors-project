#! /usr/bin/python3

import serial
import time
import array

DEFAULT_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
INTERVAL = 1

def main():
    serial_port = serial.Serial(DEFAULT_PORT, BAUDRATE, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

    frame = ""

    while True:
        try:    
            while serial_port.in_waiting > 0:
                byte = serial_port.read()

                if byte == b"\n":
                    print (frame)
                    frame = ""
                    continue

                if byte != b"\x86":
                    frame += byte.decode()
           
            time.sleep(INTERVAL)

        except KeyboardInterrupt: 
            print ("\nExit")
            break

if __name__ == "__main__":
    
    main()
