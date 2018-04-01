
#include <WaspSensorGas_Pro.h>
#include <BME280.h>
#include <WaspGPS.h>

// define GPS timeout when connecting to satellites
// this time is defined in seconds (240sec = 4minutes)
#define TIMEOUT 240

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
  USB.println(F("Gas sensors test"));

  GPS.ON();
  Utils.setLED(LED1, LED_ON);
  ///////////////////////////////////////////////////
  // 1. wait for GPS signal for specific time
  ///////////////////////////////////////////////////
  status = GPS.waitForSignal(TIMEOUT);

  if( status == true )
  {
    USB.println(F("\n----------------------"));
    USB.println(F("Connected"));
    USB.println(F("----------------------"));
  }
  else
  {
    USB.println(F("\n----------------------"));
    USB.println(F("GPS TIMEOUT. NOT connected"));
    USB.println(F("----------------------"));
  }
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
}


void loop()
{
  ///////////////////////////////////////////
  // 1. Power on  sensors
  ///////////////////////////////////////////

  // NDIR gas sensor needs a warm up time at least 120 seconds
  // To reduce the battery consumption, use deepSleep instead delay
  // After 2 minutes, Waspmote wakes up thanks to the RTC Alarm
  //PWR.deepSleep("00:00:02:00", RTC_OFFSET, RTC_ALM1_MODE1, ALL_ON);
  // Utils.setLED(LED0, LED_ON);
  // delay(60000);


  ///////////////////////////////////////////
  // 2. Read sensors
  ///////////////////////////////////////////
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
  USB.println(F("***************************************"));
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
  USB.print(F("Altitude [m]: "));
  USB.println(GPS.altitude);
  USB.print("Latitude (degrees):");
  USB.println(GPS.convert2Degrees(GPS.latitude, GPS.NS_indicator));
  USB.print("Longitude (degrees):");
  USB.println(GPS.convert2Degrees(GPS.longitude, GPS.EW_indicator));
  USB.print(F("Battery Level: "));
  USB.print(PWR.getBatteryLevel(),DEC);
  USB.print(F(" %"));
  USB.print(F(" | Battery (Volts): "));
  USB.print(PWR.getBatteryVolts());
  USB.println(F(" V"));


  ///////////////////////////////////////////
  // 3. Power off sensors
  ///////////////////////////////////////////

  // Power off the NDIR sensor. If there aren't more gas sensors powered,
  // turn off the board automatically
  // CO2.OFF();
  // SO2.OFF();
  // CO.OFF();
  // NO.OFF();
  // digitalWrite(GP_I2C_MAIN_EN, LOW);

  ///////////////////////////////////////////
  // 4. Sleep
  ///////////////////////////////////////////

  // Go to deepsleep.
  // After 3 minutes, Waspmote wakes up thanks to the RTC Alarm
  //PWR.deepSleep("00:00:02:00", RTC_OFFSET, RTC_ALM1_MODE1, ALL_OFF);
  // Utils.setLED(LED0, LED_OFF);
  //delay(1000);
}
