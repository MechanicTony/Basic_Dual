
#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"

// Booleans to see if we are using CMPS or BNO08x or Dual
bool useCMPS = false;
bool useBNO08x = false;
bool useDual = false;
bool dualReady = false;
bool dualReady1 = false;

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

/************************* User Settings *************************/
//Serial Ports
#define SerialGPS2 Serial2  //2nd F9P 10hz relPos
#define SerialGPS Serial1   //1st F9P 10hz GGA,VTG + 1074,1084,1094,1124,1230,4072.0
#define SerialAOG Serial    //AgOpen / USB 
 
//is the GGA the second sentence?
const bool isLastSentenceGGA = true;

const int32_t baudAOG = 115200;
const int32_t baudGPS = 115200;

#define RX1   27
#define TX1   16

#define RX2   25
#define TX2   17

/*****************************************************************/

 /* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

//how long after last sentence should imu sample
const uint16_t IMU_DELAY_TIME = 90;  
uint32_t lastTime = IMU_DELAY_TIME;
uint32_t currentTime = IMU_DELAY_TIME;

//how long after last time should imu sample again
const uint16_t GYRO_LOOP_TIME = 10;  
uint32_t lastGyroTime = GYRO_LOOP_TIME, lastPrint;

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
    SerialGPS.begin(baudGPS, SERIAL_8N1, RX1, TX1);
    SerialGPS.setRxBufferSize(150);
    //GPS2 Started below

    //the dash means wildcard
    parser.setErrorHandler(errorHandler);
    parser.addHandler("G-GGA", GGA_Handler);
    parser.addHandler("G-VTG", VTG_Handler);

    Wire.begin();

    pinMode(13, OUTPUT);

    //test if CMPS working
    uint8_t error;
    //    Serial.println("Checking for CMPS14");
    Wire.beginTransmission(CMPS14_ADDRESS);
    error = Wire.endTransmission();

    if (error == 0)
    {
        //      Serial.println("Error = 0");
        //      Serial.print("CMPS14 ADDRESs: 0x");
        //      Serial.println(CMPS14_ADDRESS, HEX);
        //      Serial.println("CMPS14 Ok.");
        useCMPS = true;
    }
    else
    {
        //      Serial.println("Error = 4");
        //      Serial.println("CMPS not Connected or Found"); 
    }

    if (!useCMPS)
    {
        for (int16_t i = 0; i < nrBNO08xAdresses; i++)
        {
            bno08xAddress = bno08xAddresses[i];

            //        Serial.print("\r\nChecking for BNO08X on ");
            //        Serial.println(bno08xAddress, HEX);
            Wire.beginTransmission(bno08xAddress);
            error = Wire.endTransmission();

            if (error == 0)
            {
                //          Serial.println("Error = 0");
                //          Serial.print("BNO08X ADDRESs: 0x");
                //          Serial.println(bno08xAddress, HEX);
                //          Serial.println("BNO08X Ok.");

                          // Initialize BNO080 lib        
                if (bno08x.begin(bno08xAddress))
                {
                    Wire.setClock(400000); //Increase I2C data rate to 400kHz

            // Use gameRotationVector
            bno08x.enableGyro(GYRO_LOOP_TIME);
            //bno08x.enableGameRotationVector(GYRO_LOOP_TIME-1);
            //bno08x.configureGyroIntegratedRotationVector(0x0207, GYRO_LOOP_TIME-1, 5, 0.02);
            bno08x.enableGyroIntegratedRotationVector(GYRO_LOOP_TIME-1); //Send data update every REPORT_INTERVAL in ms for BNO085, looks like this cannot be identical to the other reports for it to work...
                        
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
                        //              Serial.println("BNO08x init fails!!");
                    }
                }
                else
                {
                    //            Serial.println("BNO080 not detected at given I2C address.");
                }
            }
            else
            {
                //          Serial.println("Error = 4");
                //          Serial.println("BNO08X not Connected or Found"); 
            }
        }
    }
    
   if (!useCMPS && !useBNO08x)
    {
      SerialGPS2.begin(baudGPS, SERIAL_8N1, RX2, TX2);
      SerialGPS2.setRxBufferSize(150);
      //useDual = true;
    }
    
    delay(1000);
}

void loop()
{
    //Read incoming nmea from GPS
    if (SerialGPS.available())
        parser << SerialGPS.read();

    //Pass NTRIP etc to GPS
    if (SerialAOG.available())
        SerialGPS.write(SerialAOG.read());

    currentTime = millis();

if(!useDual){
    if (isTriggered && currentTime - lastTime >= IMU_DELAY_TIME)
    {
        //read the imu
        imuHandler();

        //reset the timer for imu reading
        isTriggered = false;
        currentTime = millis();
    }     

    if (currentTime - lastGyroTime >= GYRO_LOOP_TIME)
    {
        GyroHandler(currentTime - lastGyroTime);
    }
}

if(!useCMPS && !useBNO08x){

if(dualReady == true && dualReady1 == true) {
  BuildPANDA();
  dualReady = false;
  dualReady1 = false;
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
  //Serial.println("ACK Received! ");
    useDual = true;
    relPosDecode();
  }
  else {
  //Serial.println("ACK Checksum Failure: ");
  }
  byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
}
