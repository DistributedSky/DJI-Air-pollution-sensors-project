#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from digi.xbee.devices import XBeeDevice

PORT = "/dev/ttyUSB2"
BAUD_RATE = 19200


def main():
    print("Libelium Waspmote data receiver\n")

    device = XBeeDevice(PORT, BAUD_RATE)

    try:
        device.open()
        device.flush_queues()
        print("Waiting for data...\n")

        while True:
            xbee_message = device.read_data(300)
            if xbee_message is not None:
                # print("From %s >> %s" % (xbee_message.remote_device.get_64bit_addr(),
                #                          xbee_message.data))
                if b"<=>" in xbee_message.data:
                    data = xbee_message.data[5:]
                    print (data.decode())



    except KeyboardInterrupt: 
        print ("Exit")

    finally:
        if device is not None and device.is_open():
            device.close()


if __name__ == '__main__':
    main()
