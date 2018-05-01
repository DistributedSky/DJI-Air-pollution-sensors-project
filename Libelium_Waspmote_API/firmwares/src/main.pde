
#include <WaspSensorGas_Pro.h>
#include <BME280.h>
#include <WaspGPS.h>
#include <WaspFrame.h>

#define USB_DEBUG 0
#define SWITCH_CO2_CH4 1  //CO2 - 1, CH4 - 0

// Define GPS timeout when connecting to satellites
#define TIMEOUT 10

// Define the Waspmote ID
char node_ID[] = "DJI_01";

// define status variable for GPS connection
bool gps_status;

#if SWITCH_CO2_CH4
  Gas CO2(SOCKET_1);
#else
  Gas CH4(SOCKET_1);  
#endif
Gas SO2(SOCKET_4);
Gas CO(SOCKET_5);
Gas NO(SOCKET_6);

float temperature;  // Stores the temperature in ºC
float humidity;   // Stores the realitve humidity in %RH
float pressure;   // Stores the pressure in Pa
float concCO, concNO, concSO2;
#if SWITCH_CO2_CH4
  float concCO2;
#else
  float concCH4; 
#endif

void setup()
{
  USB.ON();
  #if USB_DEBUG
    USB.println(F("Drone Employee"));
    USB.println(F("Air pollution sensors project"));
  #endif

  pinMode(GP_I2C_MAIN_EN, OUTPUT);  // For BME sensor

  #if SWITCH_CO2_CH4
    CO2.ON();
  #else
    CH4.ON(); 
  #endif
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
  // Init GPS
  GPS.ON();
}


void loop()
{
  // 1. Read sensors

  #if SWITCH_CO2_CH4
    concCO2 = CO2.getConc();
  #else
    concCH4 = CH4.getConc();
  #endif
  concSO2 = SO2.getConc();
  concCO = CO.getConc();
  concNO = NO.getConc();

  temperature = BME.getTemperature(BME280_OVERSAMP_16X, BME280_FILTER_COEFF_OFF);
  humidity = BME.getHumidity(BME280_OVERSAMP_16X);
  pressure = BME.getPressure(BME280_OVERSAMP_16X, BME280_FILTER_COEFF_OFF);

  gps_status = GPS.waitForSignal(TIMEOUT);
  if( gps_status == true ) {
    GPS.getPosition();
  }

  // // And print the values via USB
  #if USB_DEBUG
    USB.println(F(">"));
    USB.print(F("Time [Day of week, YY/MM/DD, hh:mm:ss]: "));
    USB.println(RTC.getTime());
    #if SWITCH_CO2_CH4
      USB.print(F("CO2 concentration: "));
      USB.print(concCO2);
    #else
      USB.print(F("CH4 concentration: "));
      USB.print(concCH4);
    #endif
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
    if( gps_status == true ) {
      USB.print("Latitude (degrees):");
      USB.println(GPS.convert2Degrees(GPS.latitude, GPS.NS_indicator));
      USB.print("Longitude (degrees):");
      USB.println(GPS.convert2Degrees(GPS.longitude, GPS.EW_indicator));
    }
    else {
      USB.println(F("GPS: FAIL"));
    }
  #endif

  // 3. Create ASCII frame
  frame.createFrame(ASCII);

  frame.addSensor(SENSOR_BME_TC, temperature);
  frame.addSensor(SENSOR_BME_HUM, humidity);
  frame.addSensor(SENSOR_BME_PRES, pressure);
  #if SWITCH_CO2_CH4
    frame.addSensor(SENSOR_GASES_PRO_CO2, concCO2);
  #else
    frame.addSensor(SENSOR_GASES_PRO_CH4, concCH4);
  #endif
  frame.addSensor(SENSOR_GASES_PRO_SO2, concSO2);
  frame.addSensor(SENSOR_GASES_PRO_CO, concCO);
  print_frame();

  frame.createFrame(ASCII);
  frame.addSensor(SENSOR_GASES_PRO_NO, concNO);
  if( gps_status == true ) {
    frame.addSensor(SENSOR_GPS,
                    GPS.convert2Degrees(GPS.latitude, GPS.NS_indicator),
                    GPS.convert2Degrees(GPS.longitude, GPS.EW_indicator) );
  }
  else {
    frame.addSensor(SENSOR_GPS, 0.0, 0.0 );
  }
  // frame.addSensor(SENSOR_TIME,GPS.timeGPS);
  // frame.addSensor(SENSOR_DATE,GPS.dateGPS);
  frame.addSensor(SENSOR_TIME,RTC.getTime());
  print_frame();

}


void print_frame () {
  #if USB_DEBUG
    USB.print("> Frame (ASCII): ");
  #endif
  USB.secureBegin();
  for( uint16_t i= 0; i < frame.length ; i++ ) {
    printByte( frame.buffer[i],  0);
  }
  printByte( '\r',  0);
  printByte( '\n',  0);
  USB.secureEnd();
}
