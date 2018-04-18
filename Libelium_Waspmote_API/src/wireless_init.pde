// - The XBee module must be configured to the default baud rate (BD=7) and AP parameter (AP=2).
// - To store parameter changes after power cycles, it is needed to execute the "writeValues" function. If not, when the XBee is powered off, values do not remain in the module's memory.
// - The battery has to be connected.
// - This example can be executed in Waspmote v12 and Waspmote v15
// - Gases PRO v30: Incompatible with SOCKET_2 and SOCKET_3
// MAC mode:
// • 0: Digi Mode. 802.15.4 header + Digi header. It enables features as Discover Node and Destination Node.
// • 1: 802.15.4 without ACKs. It doesn’t support DN and ND features. It is 802.15.4 protocol without generating
// ACKs when a packet is received.
// • 2: 802.15.4 with ACKs. It doesn’t support DN and ND features. It is the standard 802.15.4 protocol.
// • 3: Digi Mode without ACKs. 802.15.4 header + Digi header. It enables features as Discover Node and Destination
// Node. It doesn’t generate ACKs when a packet is received.

#include <WaspXBee802.h>

// PAN (Personal Area Network) Identifier
uint8_t  panID[2] = {0x12,0x34};

// Define Freq Channel to be set:
// Center Frequency = 2.405 + (CH - 11d) * 5 MHz
//   Range: 0x0B - 0x1A (XBee)
//   Range: 0x0C - 0x17 (XBee-PRO)
uint8_t  channel = 0x0F;

// Define the Encryption mode: 1 (enabled) or 0 (disabled)
uint8_t encryptionMode = 0;

// Define the AES 16-byte Encryption Key
char  encryptionKey[] = "WaspmoteLinkKey!";

void setup()
{
  // init USB port
  USB.ON();
  USB.println(F("Radio test"));

  // init XBee
  xbee802.ON(SOCKET0);

  /////////////////////////////////////
  // 1. set channel
  /////////////////////////////////////
  xbee802.setChannel( channel );

  // check at commmand execution flag
  if( xbee802.error_AT == 0 )
  {
    USB.print(F("1. Channel set OK to: 0x"));
    USB.printHex( xbee802.channel );
    USB.println();
  }
  else
  {
    USB.println(F("1. Error calling 'setChannel()'"));
  }


  /////////////////////////////////////
  // 2. set PANID
  /////////////////////////////////////
  xbee802.setPAN( panID );

  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 )
  {
    USB.print(F("2. PAN ID set OK to: 0x"));
    USB.printHex( xbee802.PAN_ID[0] );
    USB.printHex( xbee802.PAN_ID[1] );
    USB.println();
  }
  else
  {
    USB.println(F("2. Error calling 'setPAN()'"));
  }

  /////////////////////////////////////
  // 3. set encryption mode (1:enable; 0:disable)
  /////////////////////////////////////
  xbee802.setEncryptionMode( encryptionMode );

  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 )
  {
    USB.print(F("3. AES encryption configured (1:enabled; 0:disabled):"));
    USB.println( xbee802.encryptMode, DEC );
  }
  else
  {
    USB.println(F("3. Error calling 'setEncryptionMode()'"));
  }

  /////////////////////////////////////
  // 4. set encryption key
  /////////////////////////////////////
  xbee802.setLinkKey( encryptionKey );

  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 )
  {
    USB.println(F("4. AES encryption key set OK"));
  }
  else
  {
    USB.println(F("4. Error calling 'setLinkKey()'"));
  }

  xbee802.setMacMode(2);

  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 )
  {
    USB.println(F("5. Mac mode set OK"));
  }
  else
  {
    USB.println(F("5. Error calling 'setMacMode()'"));
  }

  /////////////////////////////////////
  // 5. write values to XBee module memory
  /////////////////////////////////////
  xbee802.writeValues();

  // check the AT commmand execution flag
  if( xbee802.error_AT == 0 )
  {
    USB.println(F("5. Changes stored OK"));
  }
  else
  {
    USB.println(F("5. Error calling 'writeValues()'"));
  }

  USB.println(F("-------------------------------"));
}



void loop()
{
  /////////////////////////////////////
  // 1. get channel
  /////////////////////////////////////
  xbee802.getChannel();
  USB.print(F("channel: "));
  USB.printHex(xbee802.channel);
  USB.println();

  /////////////////////////////////////
  // 2. get PANID
  /////////////////////////////////////
  xbee802.getPAN();
  USB.print(F("panid: "));
  USB.printHex(xbee802.PAN_ID[0]);
  USB.printHex(xbee802.PAN_ID[1]);
  USB.println();

  /////////////////////////////////////
  // 3. get encryption mode (1:enable; 0:disable)
  /////////////////////////////////////
  xbee802.getEncryptionMode();
  USB.print(F("encryption mode: "));
  USB.printHex(xbee802.encryptMode);
  USB.println();

  xbee802.getMacMode();
  USB.print(F("mac mode: "));
  USB.printHex(xbee802.macMode);
  USB.println();
  delay(5000);
}
