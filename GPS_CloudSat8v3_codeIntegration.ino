#include <Adafruit_GPS.h> //name of GPS Library
#include <SoftwareSerial.h>
#include <ArduCAM.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "memorysaver.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <math.h>
#define LIGHT_SENSOR A0
Adafruit_BME280 bme;

SoftwareSerial mySerial (3,2); //RX2 TX3

//Camera
#if !(defined OV2640_MINI_2MP)
  #error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define SD_CS 9
const int SPI_CS = 7;
int count = 0;
#if defined (OV2640_MINI_2MP)
  ArduCAM myCAM( OV2640, SPI_CS );
#endif
  

//create GPS library Object
Adafruit_GPS GPS(&mySerial); 

//create char to read the characters as they come in
char c;

//HallEffectSensor
int outputpin = 0;
float Rsensor;

void setup()
{
  //set baud rate
  Serial.begin(115200);
  GPS.begin(9600);

  //update 10 hertz and only request RMC and GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  delay(1000);

  //Camera
  uint8_t vid, pid;
  uint8_t temp;
  Wire.begin();
  SD.begin();
  Serial.println(F("ArduCAM Start!"));
  //set the CS as an output:
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  //initialze SPI:
  SPI.begin();

  //Reset the CPLD
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  while(1) {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);

    if (temp != 0x55) {
      Serial.println(F("SPI interface Error!"));
      delay(1000); continue;
    }else {
      Serial.println(F("SPI interface OK.")); break;
    }
  }
  //Initialize SD Card
  while(!SD.begin(SD_CS)){
    Serial.println(F("SD Card Error!"));delay(1000);
  }
  Serial.println(F("SD Card detected."));

  #if defined (OV2640_MINI_2MP)
    while(1){
      //Check if the camera module type is OV2640
      myCAM.wrSensorReg8_8(0xff, 0x01);
      myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
      myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
      if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))){
        Serial.println(F("Can't find OV2640 module!"));
        delay(1000);continue;
      }
      else{
        Serial.println(F("OV2640 detected."));break;
      } 
    }
  #endif
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  #if defined (OV2640_MINI_2MP)
    myCAM.OV2640_set_JPEG_size(OV2640_640x480);
  #endif
  delay(1000);

  //BME280
  if (!bme.begin()) {
    Serial.println("Couldn't find the BME280 sensor");
    while(1);
  }
  
}


void loop() 
{
    char time_buffer[10];
    String time = getTime();
    time.toCharArray(time_buffer,10);
    Serial.println(time_buffer);
    Serial.println();
    
    char alt_buffer[10];
    float altitude = getAltitude()  * 3.28084;
    dtostrf(altitude,5,2,alt_buffer);
    Serial.println(alt_buffer);
    
    char lat_buffer[10];
    float latitude = getLatitude();
    dtostrf(latitude,7,4,lat_buffer);
    Serial.print(lat_buffer);
    Serial.print(", ");

    char lon_buffer[10];
    float longitude = getLongitude();
    dtostrf(longitude,7,4,lon_buffer);
    Serial.println(lon_buffer);   
    Serial.println();

    //Camera
    //Temporary number of pictures (3 pictures) until we determine how often to take a picture based on altitude.

    if (count < 3) {
      delay(2500);
      myCAMSaveToSDFile();
      count++;
      delay(500);
    }

    //HallEffectSensor
    int rawValue = analogRead(outputpin);
    int gValue = (5 - rawValue) / 0.0025;
    Serial.print(gValue;
    delay(500);

    //Luminosity sensor
    int sensorValue = analogRead(Rsensor);
    Serial.println("read data is");
    Serial.println(sensorValue);
    delay(1000);
}


void readGPS()
{
  //create two strings to parse the two types of NMEA Sentences
  String NMEA1;
  String NMEA2;
  
  //clear corrupted data
  clearGPS();

  while(!GPS.newNMEAreceived())
{
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());
  NMEA1 = GPS.lastNMEA();

  //immediately read next sentence

  while(!GPS.newNMEAreceived())
{
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());
  NMEA2 = GPS.lastNMEA();
}


void  clearGPS()
{
//read and parse old data to ensure non-corrupt sentences are saved
//also accounts for delays in the main loop

  while(!GPS.newNMEAreceived())
{
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());

  while(!GPS.newNMEAreceived())
{
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());

  while(!GPS.newNMEAreceived())
{
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());
}


int getAltitude()
{
  //read GPS
  readGPS();

  //no fix check
  if (GPS.fix == 0)
  {
      Serial.print("no fix");  
      return 0;
  }

  //print value 
  //Serial.print(GPS.altitude * 3.28084);
  //Serial.println(" ft");
  
  //multiply meters by 3.2 to get feet
  return GPS.altitude;    
}


float getLatitude()
{
  //read GPS
   readGPS();

  //no fix check
  if (GPS.fix == 0)
  {
      Serial.print("no fix");  
      return 0;
  }
  
  //print value 
  //Serial.println(GPS.latitudeDegrees, 4);
  
  return GPS.latitudeDegrees;    
}

float getLongitude()
{
  //read GPS
   readGPS();

  //no fix check
  if (GPS.fix == 0)
  {
      Serial.print("no fix");  
      return 0;
  }
  
  //print value 
  //Serial.print(GPS.logitudeDegrees, 4);
  
  return GPS.longitudeDegrees;  
}


String getTime()
{
  //read GPS
  readGPS();

  //define variables
  String output;
  int hour = GPS.hour - 5;
  int minute = GPS.minute;
  int second = GPS.seconds;

  //define output string
  output += hour;
  output += ":";
  output += minute;
  output += ":";
  output += second;

  //print
  //Serial.print(output); 
  //Serial.println(""); 

  return output;
}
//Camera
void myCAMSaveToSDFile(){
  char str[8];
  byte buf[256];
  static int i = 0;
  static int k = 0;
  uint8_t temp = 0,temp_last=0;
  uint32_t length = 0;
  bool is_header = false;
  File outFile;
  //Flush the FIFO
  myCAM.flush_fifo();
  //Clear the capture done flag
  myCAM.clear_fifo_flag();
  //Start capture
  myCAM.start_capture();
  Serial.println(F("start Capture"));
  while(!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK));
    Serial.println(F("Capture Done."));  
    length = myCAM.read_fifo_length();
    Serial.print(F("The fifo length is :"));
    Serial.println(length, DEC);
    if (length >= MAX_FIFO_SIZE) //384K
    {
      Serial.println(F("Over size."));
       return ;
    }
    if (length == 0 ) //0 kb
    {
      Serial.println(F("Size is 0."));
      return ;
    }
  //Construct a file name
  k = k + 1;
  itoa(k, str, 10);
  strcat(str, ".jpg");
  //Open the new file
  outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
  if(!outFile){
    Serial.println(F("File open faild"));
    return;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buf[i++] = temp;  //save the last  0XD9     
      //Write the remain bytes in the buffer
      myCAM.CS_HIGH();
      outFile.write(buf, i);    
      //Close the file
      outFile.close();
      Serial.println(F("Image save OK."));
      is_header = false;
      i = 0;
    }  
    if (is_header == true)
    { 
      //Write image data to buffer if not full
      if (i < 256)
        buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }        
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      buf[i++] = temp_last;
      buf[i++] = temp;   
    } 
  } 
}
//BME280
float getTemperature() {
  return bme.readTemperature() * 9.0/5.0 + 32;
}

float getHumidity() {
  return bme.readHumidity();
}

float getPressure() {
  return bme.readPressure();
}
