#include <SPI.h>
#include <SD.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#define DEBUG_WITH_SERIAL 0  // Set to 0 so it doesn't wait on serial to open...or slow down the program

#define SEC_PER_FLUSH 2   //Flushing data to SD card takes time...do it after this many seconds
#define SEC_PER_TEMP 5    //Temperature doesn't change that often...log it after this many seconds
#define FILE_NAME "test.csv"

//Nano Color LEDs
#define RED 22     
#define BLUE 24     
#define GREEN 23

#define SD_CARD_CHIP_SELECT_PIN 10
enum sd_state{
  NONE,
  ERR,
  READING,
  WRITING
};
 
// i2c for accelerometer
// Use 'Wire1' for the Nano Sense BLE board as it has two I2C interfaces
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(&Wire1,0);

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

void setupSensor()
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}

File myFile;
void setup() {
  // Light up the green LED when accessing the car
  pinMode(GREEN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BLUE, OUTPUT); 
  showState(NONE);

  if (DEBUG_WITH_SERIAL){
    // Open serial communications and wait for port to open:
    Serial.begin(115200);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  }

  //--------------------------------------------------
  //Initialize the accelerometer first
  Serial.println("Initializing the LSM9DS1 accelerometer");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    showState(ERR);  //Set RED to show error
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();

  //--------------------------------------------------
  //Initialize the SD Card
  showState(READING);
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CARD_CHIP_SELECT_PIN)) {
    Serial.println("initialization failed!");
    showState(ERR);  //Set RED to show error
    while (1);
  }
  myFile = SD.open(FILE_NAME, FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("File opened");
  } else{
    Serial.println("File opening failed!");
    showState(ERR);  //Set RED to show error
    while (1);
  }
  myFile.println("Ms,AccelX,AccelY,AccelZ,Temp");
  
  showState(NONE);
  
  Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
}

void showState(sd_state state) {
  switch (state){
    case NONE:
      digitalWrite(GREEN, HIGH);
      digitalWrite(RED, HIGH);
      digitalWrite(BLUE, HIGH);
      break;
    case ERR:
      digitalWrite(GREEN, HIGH);
      digitalWrite(RED, LOW);
      digitalWrite(BLUE, HIGH);
      break;      
    case READING:
      digitalWrite(GREEN, LOW);
      digitalWrite(RED, HIGH);
      digitalWrite(BLUE, HIGH);
      break;  
    case WRITING:
      digitalWrite(GREEN, HIGH);
      digitalWrite(RED, HIGH);
      digitalWrite(BLUE, LOW);
      break; 
  }
    
}

unsigned long lastFlushTime=0;
unsigned long lastTempTime=0;
unsigned long now;
unsigned long earlier;
unsigned long delta_time;
void loop() {

  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event */ 
  sensors_event_t a, temp;

  //Only grab the temp every so often to speed up the read
  if (now>lastTempTime+(SEC_PER_TEMP*1000)){
    lsm.getEvent(&a, NULL, NULL, &temp);
    lastTempTime = now;
  }else{
    lsm.getEvent(&a, NULL, NULL, NULL);
    temp.temperature = 0;
  }

  now = millis();
  delta_time = now-earlier;
  earlier = now;

  if (DEBUG_WITH_SERIAL){
    Serial.print("Ms: "); Serial.println(delta_time);
    //Serial.print("\tAccel X: "); Serial.print(a.acceleration.x);
    //Serial.print("\tY: "); Serial.print(a.acceleration.y);
    //Serial.print("\tZ: "); Serial.print(a.acceleration.z);
    //Serial.println();
  }

  showState(WRITING);
  myFile.print(delta_time); myFile.print(",");
  myFile.print(a.acceleration.x); myFile.print(",");
  myFile.print(a.acceleration.y); myFile.print(",");
  myFile.print(a.acceleration.z); 
  if (temp.temperature != 0){
     myFile.print(",");
     myFile.print(temp.temperature);
  }
  myFile.println();

  if (now>lastFlushTime+(SEC_PER_FLUSH*1000)){
    myFile.flush();
    lastFlushTime=now;
  }
  
  showState(NONE);

  /*Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" uT");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" uT");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" uT");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" rad/s");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" rad/s");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" rad/s");
*/
  //delay(10);
}
