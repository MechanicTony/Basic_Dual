# Basic_Dual
 This is a basic usb or ethernet (W5500 module) GPS code for ESP32 and AgOpenGPS  
 You can use 1xF9P and IMU (BNO08x or CMPS14), It will send PANDA to AgOpen  
 Or you can use 2xF9P and use dual anttena, It will send PAOGI to AgOpen  
   
 Notes:  
 1st F9P always send GGA/VTG to ESP @10hz, and if dual is used 1074,1084,1094,1124,1230,4072.0 to 2nd F9P   
 2nd F9P always sends relPos to ESP @10hz (If dual is used)  
 If IMU is used, pullup resistors must be added to both SDA & SCL pins  
