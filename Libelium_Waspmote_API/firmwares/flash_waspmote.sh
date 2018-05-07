#!/bin/bash

if [[ $# -lt 1 ]]; 
	then echo "Choose sensor type"
	echo "Usage:"
	echo "$0 [co2] OR [ch4]"
	exit
fi

SENST="$1"

avrdude -Cavrdude.conf -v -V -patmega1281 -cstk500v1 -P/dev/ttyUSB0 -b115200 -D -F -Uflash:w:waspmote_fw_$SENST.hex:i 
