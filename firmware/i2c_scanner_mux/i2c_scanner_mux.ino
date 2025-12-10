// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
// 
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux

QWIICMUX myMux;

void setup()
{
  Wire.begin();
  Wire.setClock(100000);      // Set the I2C SCL to 400kHz
  
  Serial.begin(115200);
  while (!Serial){ delay(10);}             // wait for serial monitor

  while (myMux.begin(0x70) == false)
  {
    Serial.println("Mux not detected. Freezing...");
    delay(5);
  }
  Serial.println("Mux detected");

  Serial.println("\nI2C Scanner");
}


void loop()
{
  byte error, address;
  int nDevices;
  byte currentPortNumber;
  
  Serial.println("Scanning...");
  for(int mux_id=0; mux_id < 8; mux_id++)
  {
    nDevices = 0;
    myMux.setPort(mux_id);
    
    currentPortNumber = myMux.getPort();
    Serial.print("CurrentPort: ");
    Serial.println(currentPortNumber);
  
    for(address = 1; address < 127; address++ ) 
    {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
  
      if (error == 0)
      {
        Serial.print("I2C device found at address 0x");
        if (address<16) 
          Serial.print("0");
        Serial.print(address,HEX);
        Serial.println("  !");
        delay(2);
        nDevices++;
      }
      else if (error==4) 
      {
        Serial.print("Unknown error at address 0x");
        if (address<16) 
          Serial.print("0");
        Serial.println(address,HEX);
      }    
    }
    if (nDevices == 0)
      Serial.println("No I2C devices found\n");
    else
      Serial.println("done\n");
  }


  

  delay(5000);           // wait 5 seconds for next scan
}
