
/*
  Bridge Sensor with GSM, SDCard, thermistor, US and RTC.

  Sketch used by MSI Sensors platform.

  Created 04JUL16
*/

#include <LowPower.h>
#include <math.h>

#include <Time.h>  /////////////Jalal Changes////////////////Beside RTCLib there must also by Time library. It includes time_t type where we can store the unix datetime type

#include <Adafruit_FONA.h> /////////////////new Changes/////////////// include fona

//#include "Sim800.h"
#include <SD.h>
#include <SPI.h>


// RTC Dependency
#include <Wire.h>
#include <RTClib.h>

// Analog Pins
#define THERMISTOR_PIN A1
#define ULTRASONIC_PIN A2

//Analog pins for HC-SR04. Block these if using Maxbotix US sensor.
int echoPin = 13;
int trigPin = 12;
int duration;
int cm;

//GSM pins
#define FONA_RX 9
#define FONA_TX 8
#define FONA_RST 4
#define FONA_RI 7
#define FONA_PWR 10

// this is a large buffer for replies
char replybuffer[255];

// Digital Pins
// Digital Pin for Maxbotix US sensor. Block if using HC-SR04.
//#define US_PIN    10
//GSM DTR power pin. Adafruit documentation says to cut the KEY to use this. Otherwise GMS always remains on.
#define GSM_PIN   5
//Thermistor power pin.
#define THERM_PIN        6
//GMS Power Key. Verify if this is needed.
//#define GSM_PWRKEY       9
//SD CS pin. Seemed to work on pin 10.
#define SD_CS_PIN        10


// Settings
// -------------------------------
//Distance from sensor to river bed in centimeters.
#define SENSOR_TO_RIVER_BED        115
//Sensor number. Jalal's unique identifier.
#define SENSOR_NUM                "2"
//Number of readings per text message.
#define SEND_DATA_AFTER_X_READINGS 1
//Each sleep cycle is approximately 8 seconds.
#define SLEEP_CYCLES               450
//Number of thermistor readings to be averaged.
#define NUM_THERM_READINGS         5
//Delay in miliseconds between temperature readings.
#define THERM_READING_DELAY        20
//Number of distance readings to be averaged.
#define NUM_DISTANCE_READINGS      3
//Delay in milliseconds between distance readings.
#define DISTANCE_READING_DELAY     200
//Character type between data points.
#define DATA_DELIM                 ';'
#define BACKUP_FILENAME            "backup.txt"
#define UNSENT_FILENAME            "unsent.txt"
#define ERROR_FILENAME             "error.txt"
// Google Voice phone number for msi.artf2@gmail.com
//#define PHONE_NUMBER               "+13027152285"
//char PHONE_NUMBER[21] = "+19072236094";////////////////////////////////new changes/////////////// phone number changed to char array since fona library dosen't support phone as as string
char PHONE_NUMBER[21] = "+13027152285"; // GoogleVoice for msi.artf2@gmail.com
#define ERROR_GSM                  "GSM Failed"
#define ERROR_SMS                  "SMS Failed"


//Fona library requirment
#include <SoftwareSerial.h>
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);


// Custom Datatypes
typedef struct {
  int distance;
  int temperature;
  //int timestamp;
  time_t timestamp; /////////Jalal Changes/////////// this should be time_t type not int
} SensorReading;


// Global Variables
int numCachedReadings = 0;
int totalReadings = 0;
SensorReading sensorReadings[SEND_DATA_AFTER_X_READINGS];
RTC_PCF8523 rtc;
//Sim800 sim800;

//SD sd;

void setup()
{
 //Turn on GSM.
  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(2000);
  
  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("FONA SMS caller ID test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));

  char message[121] = "GSM Testing!";
  
  if (!fona.sendSMS(PHONE_NUMBER, message)) { ///////////new changes ////////////// checks if message sent,
     Serial.println(F("SMS Sent"));
  } else {Serial.println(F("SMS Not Sent"));}

  // SC card CS pin defined.
  pinMode(SD_CS_PIN, OUTPUT);
  //Thermistor pin defined.
  pinMode(THERM_PIN, OUTPUT);
  // PinMode settings for HC-SR04 ultrasonic sensor. Block these if using Maxbotix US sensor.
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

 //Turn off GSM.
  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(2000);
}


void loop()
{
//Serial.println(F("Wake up loop."));
  // 1. Wake up.
  // -----------

  // Enter idle state for 8 s increments with the rest of peripherals turned off.
  for (int i = 0; i < SLEEP_CYCLES; ++i)
  {
 //  Serial.println(F("Sleep cycle loop."));
//   Serial.println(i); 
//  This is the function for power down.
//  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
//  This is the function for 32u4 idle. 
    LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, 
                  TIMER0_OFF, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF);
  }

  // 2. Turn on thermistor.
  // ----------------------
  digitalWrite(THERM_PIN, HIGH);

  // 3. Take 5 thermistor readings. (one every 20ms)
  // -----------------------------------------------
  int thermReadings[NUM_THERM_READINGS];
  for (int i = 0; i < NUM_THERM_READINGS; ++i)
  {
    thermReadings[i] = analogRead(THERMISTOR_PIN);
    delay(THERM_READING_DELAY);
  }

  // 4. Turn off thermistor.
  // -----------------------
  digitalWrite(THERM_PIN, LOW);
  delay(500);

  // 5. Average 5 thermistor readings.
  // ---------------------------------
  double sumTherm = 0;
  for (int i = 0; i < NUM_THERM_READINGS; ++i)
  {
    sumTherm += thermReadings[i];
  }
  double avgTherm = sumTherm / NUM_THERM_READINGS;
  avgTherm = 1023 / avgTherm - 1;
  double R = 10000 / avgTherm;


  // 6. Convert average thermistor reading into temperature.
  // -------------------------------------------------------

  // Steinhart-Hart, modified:
  // Measure the actual value of the 10K resistor for a more accurate reading.
  double avgTemperature = ( 3950.0 / (log( R / (10000.0 * exp( -3950.0 / 298.13 ) ) ) ) ) - 273.13;

  // 7. Turn on ultrasonic sensor
  // HC-SR04 ultrasonic sensor. Block if using Maxbotix US sensor.
  digitalWrite(trigPin, LOW);
  delay(200);
  digitalWrite(trigPin, HIGH);
  delay(200);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  cm = duration / 29 / 2;
  //avgDistance = cm;
  double avgDistance = cm; /////////////////Jalal CHanges/////////////////////avgDistance variable should be declared

  //At room temperature, sound velocity is approximately 29 cm/ms. Divide by 2 to account for return time.
  //Doing this to simplify sketch and do temperature augmentation for either US sensor with same code.
  //Only one reading with method.
  delay(3000);

  //Maxbotix US sensor. Block if using HC-SR04.
  //digitalWrite(US_PIN, HIGH);
  // Calibration time
  //delay(3000);


  // 8. Take 3 distance readings. (One every 200ms)
  // ----------------------------------------------
  //int distanceReadings[NUM_DISTANCE_READINGS];
  //for (int i = 0; i < NUM_DISTANCE_READINGS; ++i)
  //{
  //  distanceReadings[i] = analogRead(ULTRASONIC_PIN);
  //  delay(DISTANCE_READING_DELAY);
  //}


  // 9. Turn off Maxbotix ultrasonic sensor.
  // ---------------------------------------
  //digitalWrite(US_PIN, LOW);
  //delay(500);


  // 10. Average 3 distance measurements.
  // ------------------------------------
  //double sumDistance = 0.0;
  //for (int i = 0; i < NUM_DISTANCE_READINGS; ++i)
  //{
  //  sumDistance += distanceReadings[i];
  //}
  //double avgDistance = sumDistance / NUM_DISTANCE_READINGS;


  // 11. Use average temperature to calculate actual distance.
  // ---------------------------------------------------------
  double adjustedDistance = SENSOR_TO_RIVER_BED - ( ( 331.1 + .6 * avgTemperature ) / 344.5 ) * avgDistance;

  int roundedDistance = round(adjustedDistance);
  int roundedTemperature = round(avgTemperature);


  // 12. Get time from RTC Shield.
  // -----------------------------
  Wire.begin();
  rtc.begin();

  //DateTime unixTime = now.unixTime();
  DateTime   unixTime = rtc.now(); ///////////////Jalal Changes//////////////// it is rtc.now(); not now.unixTime();

// 12b. Get battery voltage and percentage
        uint16_t vbat;
        fona.getBattVoltage(&vbat);

  // 13. Combine time, distance, and temperature into a single string.
  // ----------------------------------------------------------------- i = std::stoi(line);
  //totalReadings += 1;


  //String dataString = String(totalReadings) + " " +
   //                   String(unixTime.unixtime()) + " " + ///////////Jalal Cahnges///////////////// Change the time to unix time then to string
   //                   String(roundedDistance) + " " +
   //                   String(roundedTemperature);

  // Cache distance and time in global array variable
  //sensorReadings[numCachedReadings].distance = roundedDistance;
  //sensorReadings[numCachedReadings].temperature = roundedTemperature;
  //sensorReadings[numCachedReadings].timestamp = unixTime.unixtime(); //////////Jalal Changes///////////////// Change the time to unix time then to string
  //numCachedReadings += 1;


  // 14. Turn on SD Card.
  // 15. Save data string to text file on SD card: "1234567890 1234 123" (roughly 20 characters)
  // -------------------------------------------------------------------------------------------

  // Try to turn on SD card. Should only need to be called once.
  //SD.begin();
  //  SD.write(BACKUP_FILENAME, dataString);

  //writeToSDCard(BACKUP_FILENAME, dataString); /////////Jalal Changes////////////wrtieToSDCard is a function to which we pass file name and the data we want to write to SD Card

  //delay(1000);

  // 16. Are there 4 unsent data strings?
  // 17. Yes. Send 4 unsent data strings in one SMS. Go to 18.
  // -------------------------------------
 // if (numCachedReadings == SEND_DATA_AFTER_X_READINGS)
  //{

    // 18. Prepare text message
    // ---------------------
    String textMessage = String(SENSOR_NUM) + " " +
    String(vbat) + " " +
   //                      String(sensorReadings[0].timestamp) + " " +
   String(unixTime.unixtime()) + " " +
   //                      String(sensorReadings[0].distance) + " " +
   String(roundedDistance) + " " +
   //                      String(sensorReadings[0].temperature);
   String(roundedTemperature);
//Serial.println(textMessage);
  //  DateTime startTime = sensorReadings[0].timestamp;
  //  int minutesElapsed = 0;

    // Format data to send as text message
  //  for (int i = 1; i < numCachedReadings; ++i)
    {
      //      minutesElapsed = (sensorReadings[i].timestamp - startTime) / 60;
    //  textMessage += String(DATA_DELIM) + String(minutesElapsed) + " " + String(sensorReadings[i].distance) + " " + String(sensorReadings[i].temperature);
   // }

 //Turn on GSM.
  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(2000);




    // 20. Send text message if GSM Ready

 /*   while (! Serial.available() ) {          ////////// new changes////////////
      if (fona.available()) {           ////////////////new changes////////////// check the state of the the GSM and phone

        //////////////new changes////////// the below three lines convert the message to char array sinse fona only accept message in char array
        char message[121];
        strncpy(message, textMessage.c_str(), sizeof(message));
        message[sizeof(message) - 1] = 0;


        if (!fona.sendSMS(PHONE_NUMBER, message)) { ///////////new changes ////////////// checks if message sent,
          SD.begin();
          //Serial.println(F("Failed"));
          writeToSDCard(ERROR_FILENAME, String(unixTime.unixtime()) + ": " + ERROR_SMS);
          writeToSDCard(UNSENT_FILENAME, textMessage);
        } else {
          //Serial.println(F("Sent!"));
        }
      } else { //////////////////////new changes ///////////if fona is not available it will write to error file in sd card
        //fona not abailable
        SD.begin();
        writeToSDCard(ERROR_FILENAME, String(unixTime.unixtime()) + ": " + ERROR_GSM);
        writeToSDCard(UNSENT_FILENAME, textMessage);
      }
    }
*/

//if (fona.available()) {           ////////////////new changes////////////// check the state of the the GSM and phone

        //////////////new changes////////// the below three lines convert the message to char array sinse fona only accept message in char array
        char message[121];
        strncpy(message, textMessage.c_str(), sizeof(message));
        message[sizeof(message) - 1] = 0;

//Serial.println(message);
fona.sendSMS(PHONE_NUMBER, message);
     //   if (!fona.sendSMS(PHONE_NUMBER, message)) { ///////////new changes ////////////// checks if message sent,
    //      SD.begin();
     //     writeToSDCard(BACKUP_FILENAME, textMessage);
          //Serial.println(F("Failed"));
         // writeToSDCard(ERROR_FILENAME, String(unixTime.unixtime()) + ": " + ERROR_SMS);
         // writeToSDCard(UNSENT_FILENAME, textMessage);
     //   } else {
      //   writeToSDCard(ERROR_FILENAME, String(unixTime.unixtime()) + ": " + ERROR_SMS);
      //    writeToSDCard(UNSENT_FILENAME, textMessage);
      //  }
     // }
    //    if (sim800.ensureReady() == true)
    //    {
    //      sim800.sendTextMsg(textMessage, PHONE_NUMBER);
    //      if (sim800.isTextMsgDelivered() == false)
    //      {
    //              SD.begin();
    //              // SD.write(ERROR_FILENAME, unixTime + ": " + ERROR_SMS);
    //              writeToSDCard(ERROR_FILENAME, String(unixTime.unixtime()) + ": " + ERROR_SMS);
    //
    //
    //           // SD.write(UNSENT_FILENAME, textMessage);
    //             writeToSDCard(UNSENT_FILENAME, textMessage);
    //
    //      }
    //    }
    //    else
    //    {
    //            SD.begin();
    //          //  SD.write(ERROR_FILENAME, String(unixTime) + ": " + ERROR_GSM);
    //          writeToSDCard(ERROR_FILENAME, String(unixTime.unixtime()) + ": " + ERROR_GSM);
    //         //  SD.write(ERROR_FILENAME, unixTime + ": " + ERROR_GSM);
    //        // writeToSDCard(ERROR_FILENAME, unixTime + ": " + ERROR_GSM);
    //         //  SD.write(UNSENT_FILENAME, textMessage);
    //         writeToSDCard(UNSENT_FILENAME, textMessage);
    //    }

    // Reset number of cached readings
   // numCachedReadings = 0;


    // 20. Turn off GSM.
    // -----------------

 //Turn off GSM.
  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(2000);
//  }
}
// File myFile;
//void writeToSDCard(String fileName, String data) { /////////Jalal Changes///////////This function is used to wrtie data in to a file in SD card
//  File myFile;
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
//  SD.begin();
//  myFile = SD.open(fileName, FILE_WRITE);

  // if the file opened okay, write to it:
//  if (myFile) {
 //   Serial.print("Writing to test.txt...");
  //  myFile.println(data);
    // close the file:
  //  myFile.close();
 //   Serial.println("done.");
 // } else {
    // if the file didn't open, print an error:
 //   Serial.println("error opening backup.txt");
 // }

}



