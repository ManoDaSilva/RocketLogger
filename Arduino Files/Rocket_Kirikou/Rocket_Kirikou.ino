/*
________________________________
Kirikou Rocket Logger Main code

By Manoel da Silva , under WTFPL
July 2015
________________________________

*/


//Librairies
#include <Wire.h>
#include "Arduino.h"
#include <SFE_BMP180.h>
#include <SPI.h>
#include <SD.h>
#include "I2Cdev.h"
#include "MPU6050.h"
//#include <Servo.h>

//Object inits
SFE_BMP180 pressure;
MPU6050 accelgyro;

//Global variables
double baseline; // baseline pressure
int timetoapoapsis = 6 * 1000;
int timeafteropen = 10 * 1000;
const int chipSelect = 10;

int16_t ax, ay, az;
int16_t gx, gy, gz;

//Setup routine
void setup() {
  //Pin definitions
  pinMode(9, OUTPUT); //Radio transmitter
  pinMode(3, OUTPUT); //Status LED
  digitalWrite(3, LOW); //Turn off the status LED
  
  Wire.begin(); //Init I2C bus
  Serial.begin(9600); //Init serial bus

  //Altimeter init
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
    while (1) {
      digitalWrite(3, LOW);
      delay(500);
      digitalWrite(3, HIGH);
      delay(500);
    };
  }
  baseline = getPressure();

  //Gyro init
  accelgyro.initialize();
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setFullScaleAccelRange(3); //To use the whole 16g range of the accelerometer, we will need it!
  accelgyro.setXAccelOffset(-300);


  //SD Card init
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    while (1) {
      digitalWrite(3, LOW);
      delay(500);
      digitalWrite(3, HIGH);
      delay(500);
    };
  }
  //Serial.println("card initialized.");

  //Writing some stuff to the SD card
  writeToSD("Controller Startup!");
  //writeToSD("Time, Altitude, AccX, AccY, AccZ, GyrX, GyrY, GyrZ, Temp");

}


//Loop routine
void loop() {
  //Awating launch...
  writeToSD("Standing by for launch");
  standby();

  //Liftoff!
  writeToSD("Liftoff!");
  flying();

  //Open hatch
  writeToSD("Parachute deployed");
  descent();

  //Landed
  writeToSD("Please find me!");
  beacon();

 

}

void standby() {
  digitalWrite(3, HIGH); //Turn on the LED
  tone(9, 520); //Send a tone to the transmitter, to allow tuning of the receiver
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  while (ax < 1000) { //Continuously check the accelerometer for an accel value above the fixed threshold
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  }
}

void flying() {
  digitalWrite(3, LOW); //Turn of the LED
  noTone(9); //Shut down the transmitter
  int time = millis();
  while (millis() - time < timetoapoapsis) {
    logVars();
  }
}

void descent() {
  int time = millis();  //Pretty much do the same thing
  while (millis() - time < timeafteropen) {
    logVars();
  }
};

void logVars() { //Fetchs the values from the accelerometer and the altimeters, writes everything on the SD card
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  String datastring = String(millis()) + ", " + String(getAltitude()) + ", " + String(ax) + ", " + String(ay) + ", " + String(az) + ", " + String(gx) + ", " + String(gy) + ", " + String(gz);
  writeToSD(datastring);
}

void beacon() { //Sends fasts blips to the transmitter
  while (1) {
    tone(9, 520);
    digitalWrite(3, LOW);
    delay(200);
    noTone(9);
    digitalWrite(3, HIGH);
    delay(200);
  }
}


void writeToSD(String text) { //Writes Strings to SD card
  File dataFile = SD.open("log.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println(text);
    //Print to the serial port too:
    Serial.println(text);
    dataFile.close();
  }
  else {
    Serial.println("error opening log.csv");
  }
}

double getAltitude() { //Go this from the BMP180 examples
  return pressure.altitude(getPressure(), baseline);
}

double getPressure()
{
  char status;
  double T, P, p0, a;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
        //else Serial.println("error retrieving pressure measurement\n");
      }
      //else Serial.println("error starting pressure measurement\n");
    }
    //else Serial.println("error retrieving temperature measurement\n");
  }
  //else Serial.println("error starting temperature measurement\n");
}


