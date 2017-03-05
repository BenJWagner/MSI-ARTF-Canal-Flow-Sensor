/*
  Bridge Sensor with GSM, SDCard, thermistor, US and RTC.
  Sketch used by MSI Sensors platform.
  Created 04JUL16
  Modified 04AUG16
*/
#include <ARTF_SDCard.h>
#include <SdFat.h>

#include <math.h>
#include <Time.h>
#include <Adafruit_FONA.h>
#include <LowPower.h>

#include <SPI.h>


// RTC Dependency
#include <Wire.h>
#include <RTClib.h>

// Analog Pins
#define THERMISTOR_PIN A1
#define ULTRASONIC_PIN A2

//Analog pins for HC-SR04. Block these if using Maxbotix US sensor.

int trigPin = 12;
int echoPin = 13;

int duration;
int cm;

//GSM pins
#define FONA_RX 9
#define FONA_TX 8
#define FONA_RST 4
//#define FONA_RI 7
#define FONA_PWR 11

// this is a large buffer for replies
char replybuffer[255];


//GSM DTR power pin. Adafruit documentation says to cut the KEY to use this. Otherwise GMS always remains on.
#define GSM_PIN   5
//Thermistor power pin.
#define THERM_PIN        6

// Settings
#define SENSOR_NUM               "10AT"
//Number of readings per text message.
#define SEND_DATA_AFTER_X_READINGS             3
//Each sleep cycle is approximately 8 seconds.
#define SLEEP_CYCLES               450
//Number of seconds the microcontroller will wait for the network to find
#define WAIT_FOR_NETWORK    180
//Number of thermistor readings to be averaged.
#define NUM_THERM_READINGS         3
//Delay in miliseconds between temperature readings.
#define THERM_READING_DELAY        200
//Number of distance readings to be averaged.
#define NUM_DISTANCE_READINGS      3
//Delay in milliseconds between distance readings.
#define DISTANCE_READING_DELAY     200
//Character type between data points.
#define DATA_DELIM                 ':'
//SMS backup file on SD card
#define BACKUP_FILENAME            "backup.csv"
//Unsent SMS file name on SD card
#define UNSENT_FILENAME            "unsent.csv"
//Phone number
#define PHONE_NUMBER_SMS               "+93728898056"


//SD card PIN
#define SD_CS_PIN                   10
ARTF_SDCard sd(SD_CS_PIN);


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
  String sensorNum;
  //int vbat;
  DateTime timestamp;
} SensorReadingSMS;

// Global Variables
int numCachedReadings = 0;
int totalReadings = 0;

SensorReadingSMS sensorReadingsSMS[SEND_DATA_AFTER_X_READINGS];
//Counts number of seconds the microcontroller is trying to find the network
int waitCount = 0;


//Real time clock requirement
RTC_PCF8523  rtc;


void setup()
{
  // SC card CS pin defined.
  pinMode(SD_CS_PIN, OUTPUT);
  //Thermistor pin defined.
  pinMode(THERM_PIN, OUTPUT);
  // PinMode settings for HC-SR04 ultrasonic sensor. Block these if using Maxbotix US sensor.
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Feather Fona power pin
  pinMode(FONA_PWR, OUTPUT);


  //Turn on GSM.
  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(8000);


  Serial.begin(115200);
  Serial.println(F("FONA SMS caller ID test"));
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (!fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }
  Serial.println(F("FONA is OK"));

  //Battery voltage for test message
  uint16_t vbat;
  fona.getBattPercent(&vbat);


  //date time of of test message
  DateTime   unixTime = rtc.now();
  int readingHour = unixTime.hour();
  int readingMinute = unixTime.minute();

  int readingYear   = unixTime.year();
  int readingMonth = unixTime.month();
  int readingDay = unixTime.day();

  //concatinating Test SMS time and date
  String readingTime = String(readingHour) + ":" + String(readingMinute);
  String readingDate = String(readingDay) + ":" + String(readingMonth) + ":" + String(readingYear);

  //String testMessageSMS =  String("Sensor:") + String(SENSOR_NUM) + String(" BV: ")  + String(vbat) + String("%") + String("Time:") + String(readingHour) + String(":") + String(readingMinute);
  //formating contents for SMS
  String testMessageSMS =  String("Sensor:") + String(SENSOR_NUM) + String(" BV: ")  + String(vbat) + String("%") + String("Time:") + String(readingTime) + String("Date:") + String(readingDate);

  //SMS requirment
  char messageSMS[121];
  //converting SMS from string to chars
  strncpy(messageSMS, testMessageSMS.c_str(), sizeof(messageSMS));
  messageSMS[sizeof(messageSMS) - 1] = 0;

  //Network status, Status 1 mean newtwork found.
  uint8_t n = fona.getNetworkStatus();

  //Search until SIM finds the network in the specific time. if not found in specific time it will jump out of the while loop
  while (n != 1) {
    n = fona.getNetworkStatus();
    if (waitCount >= WAIT_FOR_NETWORK)
    {
      //No network found in the defined time the MC will jump to loop
      break;
    }
    else {
      //add five seconds with network wait time
      waitCount += 5;
    }
    //wait five seconds for the network
    delay(5000);
  }

  //wait for the SIM to be ready
  delay(5000);

  //send SMS
  fona.sendSMS(PHONE_NUMBER_SMS, messageSMS);

  //Turn off GSM.
  digitalWrite(FONA_PWR, HIGH);
  delay(2000);
  digitalWrite(FONA_PWR, LOW);
  delay(2000);

}


void loop()
{

  //Sleep the MC
  for (int i = 0; i < SLEEP_CYCLES; ++i)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    digitalWrite(FONA_PWR, HIGH);  //keeps the GSM off
  }



  //Take temperature and distance reading
  double temperature = takeThermReading();
  double distance = takeDistanceReading(temperature);

  //rounding temperature and distance reading
  int roundedTemperature = round(temperature);
  int roundedDistance = round(distance);



  //RTC requirment
  Wire.begin();
  rtc.begin();

  //Date formating
  DateTime   unixTime = rtc.now();


  // Cache distance, temp and time in global array variable
  sensorReadingsSMS[numCachedReadings].sensorNum = SENSOR_NUM;
  sensorReadingsSMS[numCachedReadings].timestamp = unixTime.unixtime();
  sensorReadingsSMS[numCachedReadings].distance = roundedDistance;
  sensorReadingsSMS[numCachedReadings].temperature = roundedTemperature;

  //increment array index
  numCachedReadings += 1;



  //Look if number of readings in the Cache array is equal to the specefied number, in our case 3
  if (numCachedReadings == SEND_DATA_AFTER_X_READINGS)
  {
    //reseting wait count to 0
    waitCount = 0;

    //Turn on GSM.
    digitalWrite(FONA_PWR, HIGH);
    delay(2000);
    digitalWrite(FONA_PWR, LOW);
    delay(15000);

    //preparing FONA
    fonaSerial->begin(4800);
    if (! fona.begin(*fonaSerial)) {
      Serial.println(F("Couldn't find FONA"));
      while (1);
    }


    //Network status, Status 1 mean newtwork found.
    uint8_t n = fona.getNetworkStatus();

    //Search until SIM finds the network in the specific time. if not found in specific time it will jump out of the while loop
    while (n != 1) {
      n = fona.getNetworkStatus();
      if (waitCount >= WAIT_FOR_NETWORK)
      {
        //No network found in the defined time the MC will jump to loop
        break;
      }
      else {
        //add five seconds with network wait time
        waitCount += 5;
      }
      //wait five seconds for the network
      delay(5000);
    }

    //wait for the SIM to be ready
    delay(5000);

    //Battery voltage
    uint16_t vbat;
    fona.getBattPercent(&vbat);

    //Preparing SMS format using cache array index 0
    String textMessageSMS = String(sensorReadingsSMS[0].sensorNum) + " " +
                            String(vbat) + " " +
                            String(sensorReadingsSMS[0].timestamp.unixtime()) + " " +
                            String(sensorReadingsSMS[0].distance) + " " +
                            String(sensorReadingsSMS[0].temperature);

    //concatinating SMS reading 1 and 2 with 0
    for (int i = 1; i < numCachedReadings; ++i)
    {
      textMessageSMS += String(DATA_DELIM) + String(sensorReadingsSMS[i].sensorNum) + " " + String(vbat) + " " + String(sensorReadingsSMS[i].timestamp.unixtime()) + " " + String(sensorReadingsSMS[i].distance) + " " + String(sensorReadingsSMS[i].temperature);
    }


    char messageSMS[121];
    strncpy(messageSMS, textMessageSMS.c_str(), sizeof(messageSMS));
    messageSMS[sizeof(messageSMS) - 1] = 0;

    //SD card
    sd.begin();
    
    //Sending SMS and writing to the log files on SD card
    if (!fona.sendSMS(PHONE_NUMBER_SMS, messageSMS)) {
      sd.writeFile(UNSENT_FILENAME, messageSMS);
    } else {
      sd.writeFile(BACKUP_FILENAME, messageSMS);
    }

    //Turn off GSM.
    digitalWrite(FONA_PWR, HIGH);
    delay(2000);
    digitalWrite(FONA_PWR, LOW);
    delay(2000);

    // Reset number of cached readings
    numCachedReadings = 0;


  }
}



//Taking temperature
double takeThermReading()
{
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
  double avgTemperature = ( 3950.0 / (log( R / (10000.0 * exp( -3950.0 / 298.13 ) ) ) ) ) - 273.13;

  return avgTemperature;
}

//taking distance reading
double takeDistanceReading(double temperature)
{


  // 8. Take 3 distance readings. (One every 200ms)
  // ----------------------------------------------
  int distanceReadings[NUM_DISTANCE_READINGS];
  for (int i = 0; i < NUM_DISTANCE_READINGS; ++i)
  {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    distanceReadings[i] = pulseIn(echoPin, HIGH);
    //Only for US sensor
    //distanceReadings[i] = analogRead(ULTRASONIC_PIN) * DISTANCE_INCREMENT;
    delay(DISTANCE_READING_DELAY);
  }


  double sumDistance = 0.0;
  for (int i = 0; i < NUM_DISTANCE_READINGS; ++i)
  {
    sumDistance += distanceReadings[i];
  }

  //averaging and converting to CM
  double avgDistance = (sumDistance / NUM_DISTANCE_READINGS) / 29.1 / 2;

  if (avgDistance >= 420) {

    avgDistance = 420;
  }


  // 11. Use average temperature to calculate actual distance.
  // ---------------------------------------------------------
  double adjustedDistance = ( ( 331.1 + .6 * temperature ) / 344.5 ) * avgDistance;


  if (adjustedDistance <= 2 )
  {
    adjustedDistance = 2;
  }
  return adjustedDistance;
}

