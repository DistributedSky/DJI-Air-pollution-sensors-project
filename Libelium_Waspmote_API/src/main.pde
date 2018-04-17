
#include <WaspSensorGas_Pro.h>
#include <BME280.h>
#include <WaspGPS.h>
#include <WaspFrame.h>
#include <WaspXBee802.h>

// define GPS timeout when connecting to satellites
// this time is defined in seconds (240sec = 4minutes)
#define TIMEOUT 0

// Define BROADCAST MAC address
char RX_ADDRESS[] = "000000000000FFFF";
// Define the Waspmote ID
char node_ID[] = "DJI_01";

// define variable
uint8_t error;

// define status variable for GPS connection
bool status;

Gas CO2(SOCKET_1);
Gas SO2(SOCKET_4);
Gas CO(SOCKET_5);
Gas NO(SOCKET_6);

float temperature;  // Stores the temperature in ºC
float humidity;   // Stores the realitve humidity in %RH
float pressure;   // Stores the pressure in Pa
float concCO, concNO, concCO2, concSO2;

void setup()
{
  USB.ON();
  USB.println(F("Drone Employee \nAir pollution sensors project"));

  GPS.ON();
  Utils.setLED(LED1, LED_ON);
  // Wait for GPS signal for specific time
  status = GPS.waitForSignal(TIMEOUT);

  if( status == true )  USB.println(F("> GPS ok"));
  else                  USB.println(F("> GPS fail"));

  Utils.setLED(LED1, LED_OFF);

  pinMode(GP_I2C_MAIN_EN, OUTPUT);  // For BME sensor
  CO2.ON();
  SO2.ON();
  CO.ON();
  NO.ON();
  digitalWrite(GP_I2C_MAIN_EN, HIGH);
  BME.ON();

  RTC.ON();
  // Setting time [yy:mm:dd:dow:hh:mm:ss]
  RTC.setTime("18:01:01:06:00:00:00");
  Utils.setLED(LED0, LED_ON);

  // Set the Waspmote ID
  frame.setID(node_ID);
  // init XBee
  xbee802.ON();
}


void loop()
{
  // 1. Read sensors
  if( status == true )
  {
    GPS.getPosition();
  }
  // Read the NDIR sensor and compensate with the temperature internally
  concCO2 = CO2.getConc();
  concSO2 = SO2.getConc();
  concCO = CO.getConc();
  concNO = NO.getConc();

  // Read enviromental variables
  temperature = BME.getTemperature(BME280_OVERSAMP_16X, BME280_FILTER_COEFF_OFF);
  humidity = BME.getHumidity(BME280_OVERSAMP_16X);
  pressure = BME.getPressure(BME280_OVERSAMP_16X, BME280_FILTER_COEFF_OFF);

  // And print the values via USB
  USB.println(F(">"));
  USB.print(F("Time [Day of week, YY/MM/DD, hh:mm:ss]: "));
  USB.println(RTC.getTime());
  USB.print(F("CO2 concentration: "));
  USB.print(concCO2);
  USB.println(F(" ppm"));
  USB.print(F("SO2 concentration: "));
  USB.print(concSO2);
  USB.println(F(" ppm"));
  USB.print(F("CO concentration: "));
  USB.print(concCO);
  USB.println(F(" ppm"));
  USB.print(F("NO concentration: "));
  USB.print(concNO);
  USB.println(F(" ppm"));
  USB.print(F("Temperature: "));
  USB.print(temperature);
  USB.println(F(" Celsius degrees"));
  USB.print(F("RH: "));
  USB.print(humidity);
  USB.println(F(" %"));
  USB.print(F("Pressure: "));
  USB.print(pressure);
  USB.println(F(" Pa"));
  USB.print("Latitude (degrees):");
  USB.println(GPS.convert2Degrees(GPS.latitude, GPS.NS_indicator));
  USB.print("Longitude (degrees):");
  USB.println(GPS.convert2Degrees(GPS.longitude, GPS.EW_indicator));

  // 3. Create ASCII frame
  frame.createFrame(ASCII);

  frame.addSensor(SENSOR_BME_TC, temperature);
  frame.addSensor(SENSOR_BME_HUM, humidity);
  frame.addSensor(SENSOR_BME_PRES, pressure);
  frame.addSensor(SENSOR_GASES_PRO_CO2, concCO2);
  frame.addSensor(SENSOR_GASES_PRO_SO2, concSO2);
  frame.addSensor(SENSOR_GASES_PRO_CO, concCO);
  send_frame();

  frame.createFrame(ASCII);
  frame.addSensor(SENSOR_GASES_PRO_NO, concNO);
  frame.addSensor(SENSOR_GPS,
                  GPS.convert2Degrees(GPS.latitude, GPS.NS_indicator),
                  GPS.convert2Degrees(GPS.longitude, GPS.EW_indicator) );
  // frame.addSensor(SENSOR_TIME,GPS.timeGPS);
  // frame.addSensor(SENSOR_DATE,GPS.dateGPS);
  frame.addSensor(SENSOR_TIME,RTC.getTime());
  send_frame();

}

void send_frame () {
  // send XBee packet
  error = xbee802.send( RX_ADDRESS, frame.buffer, frame.length );
  // check TX flag
  if( error == 0 )
  {
    USB.println(F("> Send ok"));
    Utils.blinkGreenLED();
  }
  else
  {
    USB.println(F("> Send fail"));
    Utils.blinkRedLED();
  }
}
