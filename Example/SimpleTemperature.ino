/*
    -This is open source driver for measuring temprature with DS18xxx family
    -It can be used and modfied as per requirement
    -It is simplified version of DS18 driver just to measure temperature
    
    Author: Ahmad Muazam
    
*/ 

#include "src/T_Sensor.h"

// Declaring some Global variables
  byte i;
  byte present = 0;
  byte type_s;              // To use the type of Temperature sensor for conversion
  byte data[9];
  byte addr[8];
float c, f;

// Instance of Class T_Sensor
T_sensor ds(5); // This constructor taking an argument that is the pin number where the sensor is attached


void setup(){
    // Initialize the serial communication
    Serial.begin(115200);

     Serial.print("ROM =");
// Find the type of sensors for correct mapping mapping
    if ( !ds.search(addr)) {
    ds.reset_search();
    delay(250);
    return;
  }
    for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Sensor Error");
      return;
  } 

}

void loop(){

    present = ds.celcius_read(type_s,addr);         // Measure the temperature in Celcius
    Serial.print("Temperature in Celcius is : ");
    Serial.print(present);

    Serial.println("  ");
    delay(1000);
    present = ds.fahrenheit_read(type_s,addr);      // Measure the temperature in Fahrenheit
    Serial.print("Temperature in Fahrenheit is : ");
    Serial.println(present);

    delay(1000);
}
