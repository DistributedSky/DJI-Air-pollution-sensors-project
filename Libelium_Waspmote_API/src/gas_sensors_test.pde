
#include <WaspSensorGas_Pro.h>
#include <BME280.h>

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
    USB.println(F("Gas sensors test"));
    pinMode(GP_I2C_MAIN_EN, OUTPUT);  // For BME sensor
}


void loop()
{
    ///////////////////////////////////////////
    // 1. Power on  sensors
    ///////////////////////////////////////////

    CO2.ON();
    SO2.ON();
  	CO.ON();
  	NO.ON();
    digitalWrite(GP_I2C_MAIN_EN, HIGH);
    BME.ON();
    
    // Show the remaining battery level
    USB.print(F("Battery Level: "));
    USB.print(PWR.getBatteryLevel(),DEC);
    USB.print(F(" %"));

    // Show the battery Volts
    USB.print(F(" | Battery (Volts): "));
    USB.print(PWR.getBatteryVolts());
    USB.println(F(" V"));

    // NDIR gas sensor needs a warm up time at least 120 seconds
    // To reduce the battery consumption, use deepSleep instead delay
    // After 2 minutes, Waspmote wakes up thanks to the RTC Alarm
    PWR.deepSleep("00:00:02:00", RTC_OFFSET, RTC_ALM1_MODE1, ALL_ON);


    ///////////////////////////////////////////
    // 2. Read sensors
    ///////////////////////////////////////////

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


    ///////////////////////////////////////////
    // 3. Power off sensors
    ///////////////////////////////////////////

    // Power off the NDIR sensor. If there aren't more gas sensors powered,
    // turn off the board automatically
    CO2.OFF();
    SO2.OFF();
    CO.OFF();
    NO.OFF();
    digitalWrite(GP_I2C_MAIN_EN, LOW);

    ///////////////////////////////////////////
    // 4. Sleep
    ///////////////////////////////////////////

    // Go to deepsleep.
    // After 3 minutes, Waspmote wakes up thanks to the RTC Alarm
    PWR.deepSleep("00:00:02:00", RTC_OFFSET, RTC_ALM1_MODE1, ALL_OFF);

}
