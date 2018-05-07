
#include <WaspSensorGas_Pro.h>
#include <BME280.h>
#include <WaspGPS.h>

#define SWITCH_CO2_CH4 1  //CO2 - 1, CH4 - 0
// Define GPS timeout when connecting to satellites
#define TIMEOUT 10
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

float temperature;
float humidity;
float pressure;
float concCO, concNO, concSO2;
float concCO2, concCH4;

void setup()
{
  USB.ON();
  pinMode(GP_I2C_MAIN_EN, OUTPUT);  // For BME sensor

  #if SWITCH_CO2_CH4
    CO2.ON();
    concCH4 = -1;
  #else
    CH4.ON(); 
    concCO2 = -1;
  #endif
  SO2.ON();
  CO.ON();
  NO.ON();
  digitalWrite(GP_I2C_MAIN_EN, HIGH);
  BME.ON();

  RTC.ON();
  // Setting time [yy:mm:dd:dow:hh:mm:ss]
  //RTC.setTime("18:01:01:06:00:00:00");
  Utils.setLED(LED0, LED_ON);
  // Init GPS
  GPS.ON();
}

void loop()
{
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

  USB.println(F(">>>"));
  USB.print(F("CO2: "));
  USB.print(concCO2);
  USB.println(F(" ppm"));
  USB.print(F("CH4: "));
  USB.print(concCH4);
  USB.println(F(" ppm"));
  USB.print(F("SO2: "));
  USB.print(concSO2);
  USB.println(F(" ppm"));
  USB.print(F("CO: "));
  USB.print(concCO);
  USB.println(F(" ppm"));
  USB.print(F("NO: "));
  USB.print(concNO);
  USB.println(F(" ppm"));
  USB.print(F("Temperature: "));
  USB.print(temperature);
  USB.println(F(" Celsius"));
  USB.print(F("RH: "));
  USB.print(humidity);
  USB.println(F(" %"));
  USB.print(F("Pressure: "));
  USB.print(pressure);
  USB.println(F(" Pa"));
  if (gps_status == true) {
    USB.print("Latitude: ");
    USB.println(GPS.convert2Degrees(GPS.latitude, GPS.NS_indicator));
    USB.print("Longitude: ");
    USB.println(GPS.convert2Degrees(GPS.longitude, GPS.EW_indicator));
    USB.print("GPS time: ");
    USB.println(GPS.timeGPS);
    USB.print("GPS date: ");
    USB.println(GPS.timeGPS);
  }
  else {
    USB.print("Latitude: ");
    USB.println("-1");
    USB.print("Longitude: ");
    USB.println("-1");
    USB.print("GPS time: ");
    USB.println("-1");
    USB.print("GPS date: ");
    USB.println("-1");
  }
  USB.print("$");
}
