// - The XBee module must be configured to the default baud rate (BD=7) and AP parameter (AP=2).
// - To store parameter changes after power cycles, it is needed to execute the "writeValues" function. If not, when the XBee is powered off, values do not remain in the module's memory.
// - The battery has to be connected.
// - This example can be executed in Waspmote v12 and Waspmote v15
// - Gases PRO v30: Incompatible with SOCKET_2 and SOCKET_3

#include <WaspXBee802.h>

void setup()
{
  // init USB port
  USB.ON();
  USB.println(F("Radio test"));

  // init XBee
  xbee802.ON(SOCKET0);
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
  delay(5000);
}
