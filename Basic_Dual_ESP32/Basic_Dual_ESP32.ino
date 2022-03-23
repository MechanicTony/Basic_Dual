/*
 * This is a simple USB only GPS code for AgOpen GPS
 * It can used as single anttena with IMU and send PANDA to AgOpen
 * Or use two F9P and send PAOGI to AgOpen
 */

/************************* User Settings *************************/
#define deBugPin   23
bool deBug = false;

//Serial Ports
#define SerialGPS Serial1   //1st F9P 10hz GGA,VTG + 1074,1084,1094,1124,1230,4072.0
#define RX1   27
#define TX1   16
#define SerialGPS2 Serial2  //2nd F9P 10hz relPos
#define RX2   25
#define TX2   17
const int32_t baudGPS = 115200;

#define SerialAOG Serial    //AgOpen / USB
const int32_t baudAOG = 115200; 
 
//is the GGA the second sentence?
const bool isLastSentenceGGA = true;

//I2C pins, SDA = 21, SCL = 22
//Note - Pullup resistors will be needed on both SDA & SCL pins

//Swap BNO08x roll & pitch?
const bool swapRollPitch = false;
//const bool swapRollPitch = true;

//BNO08x, time after last GPS to load up IMU ready data for the next Panda takeoff
const uint16_t IMU_DELAY_TIME = 90; //Best results seem to be 90-95ms
uint32_t IMU_lastTime = IMU_DELAY_TIME;
uint32_t IMU_currentTime = IMU_DELAY_TIME;

//BNO08x, how offen should we get data from IMU (The above will just grab this data without reading IMU)
const uint16_t GYRO_LOOP_TIME = 10;  
uint32_t lastGyroTime = GYRO_LOOP_TIME;

//CMPS14, how long should we wait with GPS before reading data from IMU then takeoff with Panda
const uint16_t CMPS_DELAY_TIME = 4;  //Best results seem to be around 5ms
uint32_t gpsReadyTime = CMPS_DELAY_TIME;

/*****************************************************************/
 
#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"

// Booleans to see if we are using CMPS or BNO08x or Dual
bool useCMPS = false;
bool useBNO08x = false;
bool useDual = false;
bool GGAReady = false;
bool relPosReady = false;

//CMPS always x60
#define CMPS14_ADDRESS 0x60 

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A,0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

//Dual 
double headingcorr = 900;  //90deg heading correction (90deg*10)

float baseline;
double baseline2;
float rollDual;
float rollDualRaw;
double relPosD;
double relPosDH;
double heading = 0;

byte CK_A = 0, CK_B = 0;
byte incoming_char;
boolean headerReceived = false;
unsigned long ackWait = millis();
byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
int i = 0;

 /* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

bool isTriggered = false, blink;

//100hz summing of gyro
float gyro, gyroSum;
float lastHeading;

float roll, rollSum;
float pitch, pitchSum;

float bno08xHeading = 0;
int16_t bno08xHeading10x = 0;

void setup()
{
    SerialAOG.begin(baudAOG);
    SerialGPS.setRxBufferSize(512);
    SerialGPS.begin(baudGPS, SERIAL_8N1, RX1, TX1);
    SerialGPS.setRxBufferSize(300);
    //GPS2 Started below

    //the dash means wildcard
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);

    Wire.begin();
    delay(500);
    pinMode(13, OUTPUT);
    pinMode(deBugPin, INPUT_PULLUP);
    deBug = !digitalRead(deBugPin);
    Serial.println();
    
    //test if CMPS working
    uint8_t error;
    if(deBug) Serial.println("Checking for CMPS14");
    Wire.beginTransmission(CMPS14_ADDRESS);
    error = Wire.endTransmission();

    if (error == 0)
    {
        if(deBug) {
          Serial.println("Error = 0");
          Serial.print("CMPS14 ADDRESs: 0x");
          Serial.println(CMPS14_ADDRESS, HEX);
          Serial.println("CMPS14 Ok.");
        } 
        useCMPS = true;
    }
    else
    {
        if(deBug) {
          Serial.println("Error = 4");
          Serial.println("CMPS not Connected or Found"); 
        }
    }

    if (!useCMPS)
    {
        for (int16_t i = 0; i < nrBNO08xAdresses; i++)
        {
            bno08xAddress = bno08xAddresses[i];
          if(deBug) {
            Serial.print("\r\nChecking for BNO08X on ");
            Serial.println(bno08xAddress, HEX);
          }
            Wire.beginTransmission(bno08xAddress);
            error = Wire.endTransmission();

            if (error == 0)
            {
              if(deBug) {
                Serial.println("Error = 0");
                Serial.print("BNO08X ADDRESs: 0x");
                Serial.println(bno08xAddress, HEX);
                Serial.println("BNO08X Ok.");
              }
                          // Initialize BNO080 lib        
                if (bno08x.begin(bno08xAddress))
                {
                    Wire.setClock(400000); //Increase I2C data rate to 400kHz

            // Use gameRotationVector
            bno08x.enableGyro(GYRO_LOOP_TIME);
            bno08x.enableGameRotationVector(GYRO_LOOP_TIME-1);
                       
            // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
            if (bno08x.getFeatureResponseAvailable() == true)
            {
              if (bno08x.checkReportEnable(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, (GYRO_LOOP_TIME-1)) == false) bno08x.printGetFeatureResponse();
              if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, (GYRO_LOOP_TIME-1)) == false) bno08x.printGetFeatureResponse();

              // Break out of loop
              useBNO08x = true;
              break;
            }
                    else
                    {
                        if(deBug) Serial.println("BNO08x init fails!!");
                    }
                }
                else
                {
                    if(deBug) Serial.println("BNO080 not detected at given I2C address.");
                }
            }
            else
            {
                if(deBug) {
                  Serial.println("Error = 4");
                  Serial.println("BNO08X not Connected or Found"); 
                }    
            }
        }
    }
    
   if (!useCMPS && !useBNO08x)
    {
      SerialGPS2.begin(baudGPS, SERIAL_8N1, RX2, TX2);
      SerialGPS2.setRxBufferSize(150);
      //useDual = true;
    }
    Serial.println();
    Serial.println("Basic Dual or Single GPS for AgOpenGPS"); 
    Serial.println("Setup done, waiting for GPS Data....."); 
    Serial.println();
    delay(2000);
}

void loop()
{
    //Read incoming nmea from GPS
    if (SerialGPS.available())
        parser << SerialGPS.read();

    //Pass NTRIP etc to GPS
    if (SerialAOG.available())
        SerialGPS.write(SerialAOG.read());

    deBug = !digitalRead(deBugPin);
    IMU_currentTime = millis();

if(!useDual){
  
  if (useBNO08x)
    {  
      if (isTriggered && IMU_currentTime - IMU_lastTime >= IMU_DELAY_TIME)
      {
        //Load up BNO08x data from gyro loop ready for takeoff
        imuHandler();

        //reset the timer 
        isTriggered = false;
      }      
    }

  if (useCMPS)
    { 
      if (isTriggered && IMU_currentTime - gpsReadyTime >= CMPS_DELAY_TIME)
      {
        imuHandler(); //Get data from CMPS (Heading, Roll, Pitch) and load up ready for takeoff
        BuildPANDA(); //Send Panda

        //reset the timer 
        isTriggered = false;
      }
    }  

  IMU_currentTime = millis();    

  if (IMU_currentTime - lastGyroTime >= GYRO_LOOP_TIME)
    {
        GyroHandler(IMU_currentTime - lastGyroTime);
    }
    
}//End Not Dual

if(!useCMPS && !useBNO08x){

if(GGAReady == true && relPosReady == true) {
  BuildPANDA();
  GGAReady = false;
  relPosReady = false;
}
  
    if (SerialGPS2.available()) {
    incoming_char = SerialGPS2.read();
    if (i < 4 && incoming_char == ackPacket[i]) {
      i++;
    }
    else if (i > 3) {
      ackPacket[i] = incoming_char;
      i++;
    }
  }
  if (i > 71) {
    checksum();
    i = 0;
  }
 } //Dual

} //Loop

void checksum() {
  CK_A = 0;
  CK_B = 0;
  for (i = 2; i < 70 ; i++) {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  if (CK_A == ackPacket[70] && CK_B == ackPacket[71]) {
  if(deBug) Serial.println("ACK Received! ");
    useDual = true;
    relPosDecode();
  }
  else {
  if(deBug) Serial.println("ACK Checksum Failure: ");
  }
  byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}
