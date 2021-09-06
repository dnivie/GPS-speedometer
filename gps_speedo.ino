#include <SoftwareSerial.h> 
//#include <TinyGPS.h> 
#include "TinyGPS++.h"
#include <U8g2lib.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// Uncomment the type of screen that is in use
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // 1.3" screen
//U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_MIRROR, /* reset=*/ U8X8_PIN_NONE);  // 1.3" screen mirrored for HUD

//U8G2_SSD1309_128X64_NONAME0_1_4W_SW_SPI u8g2(U8G2_R0, 13, 11, 10, 9, 8);    // 2.4" screen
//U8G2_SSD1309_128X64_NONAME0_1_4W_SW_SPI u8g2(U8G2_MIRROR, 13, 11, 10, 9, 8);    // 2.4" screen mirrored for HUD
// UNO/Mega: scl 13, sda 11, res 8, dc 9, cs 10

/*
gps speedometer
HUD
kompass
*/
int RX = 2, TX = 3;
SoftwareSerial ss(RX, TX);
TinyGPSPlus gps;

//float lat = 59.9134,lon = 10.7549; // create variable for latitude and longitude object  
float lat1 = 0.0000, lon1 = 0.0000;
float lat2 = 0.0000, lon2 = 0.0000;
SoftwareSerial gpsSerial(3,4); //rx,tx 

//TinyGPS gps; // create gps object 

unsigned long startMillis;  //timer
unsigned long currentMillis;
const unsigned long period = 100;  // read sensor interval 500 ms

void setup(){ 
  Serial.begin(9600); // connect serial 
  //Serial.println("The GPS Received Signal:"); 
  //gpsSerial.begin(9600); // connect gps sensor 
  //gpssoft.begin(9600);
  u8g2.begin();
  startMillis = millis(); //start timer
  
} 


void loop(){
  //int directionNr = 1;
  //double speed_kmh = 0;
  //char lastDirection[5] = {'N', 'S', 'E', 'W'};
  currentMillis = millis();
  
  if (currentMillis - startMillis >= period){
  while (ss.available() > 0)
      if (gps.encode(ss.read()))
  
  if (gps.location.isUpdated())
  {
    Serial.print("LAT="); Serial.print(gps.location.lat(), 6);
    Serial.print("LNG="); Serial.println(gps.location.lng(), 6);
    Serial.println(gps.date.day());
    Serial.println(gps.date.month());
    Serial.println(gps.date.year());
    Serial.println(gps.time.value());
    Serial.println(gps.course.deg());
    Serial.println(gps.altitude.meters());
    Serial.println(gps.satellites.value());
  }
  
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println("error");
    while (true);
  }
  /*
  if (currentMillis - startMillis >= period){
    // read gps
    
    while(gpsSerial.available()){ // check for gps data 
    if( gps.encode(gpsSerial.read()) ){ // encode gps data   
      gps.f_get_position(&lat2,&lon2); // get latitude and longitude 
    } 
    
    
    speed_kmh = speed_calculation(lat1, lon1, lat2, lon2);
    directionNr = compass(lat1, lon1, lat2, lon2);
    lat1 = lat2;  // set previous position
    lon1 = lat2;
    // nr.2 is the latest reading
  }*/

  //String latitude = String(lat,6); 
  //String longitude = String(lon,6); 
  //Serial.println(latitude+";"+longitude);
    }
  

  // display
  u8g2.firstPage();
  do {

    u8g2.setFont(u8g2_font_fub30_tf);
    char cstr[6];
    if (gps.speed.isValid()){
      dtostrf(gps.speed.kmph(), 1, 1, cstr);
      u8g2.drawStr(0, 39, cstr);  
    }
    else{
      u8g2.drawStr(0, 39, "no data");
    }
    u8g2.setFont(u8g2_font_fub11_tf);
    u8g2.drawStr(0, 60, "km/h");
    //char cstr1[2];
    //dtostrf(lastDirection[directionN], 1, 1, cstr1);
    //u8g2.drawStr(80, 60, cstr1);
    //u8g2.drawStr(80, 60, &lastDirection[directionNr]);
    //Serial.println(&lastDirection[directionNr]);
    
    
  } while( u8g2.nextPage() );
}


double speed_calculation(double latitude1, double longitude1, double latitude2, double longitude2)
{
  /*
    trigonometri:
    tanx = mot/hos
    sinx = mot/hyp
    cosx = hos/mot
    pythagoras:
    c = sqrt(a^2 + b^2)
  */
  
  // Convert degrees to radians
	latitude1 = latitude1 * M_PI / 180.0;
	longitude1 = longitude1 * M_PI / 180.0;
 
	latitude2 = latitude2 * M_PI / 180.0;
	longitude2 = longitude2 * M_PI / 180.0;

  // radius of earth in metres
	double r = 6378100;
 
	// P
	double rho1 = r * cos(lat1);
	double z1 = r * sin(lat1);
	double x1 = rho1 * cos(lon1);
	double y1 = rho1 * sin(lon1);
 
	// Q
	double rho2 = r * cos(lat2);
	double z2 = r * sin(lat2);
	double x2 = rho2 * cos(lon2);
	double y2 = rho2 * sin(lon2);
 
	// Dot product
	double dot = (x1 * x2 + y1 * y2 + z1 * z2);
	double cos_theta = dot / (r * r);
 
	double theta = acos(cos_theta);

  // Distance in Metres
	float meters = float(r * theta);
  float mps = meters / (period/1000.0); 
  float kmh = (mps * 3600.0) / 1000.0;

  return kmh;
}

int compass(double latitude1, double longitude1, double latitude2, double longitude2) 
{
  double deltaLat = 0; // change in latitude
  double deltaLon = 0; // change in longitude
  //char directionLat;
  //char directionLon;
  int mainDir;
  //int secDir;
  int lastDirection;

  deltaLat = latitude2 - latitude1;
  deltaLon = longitude2 - longitude1;

  if(abs(deltaLat) > abs(deltaLon)){ // main direction change in latitude
    if(deltaLat > 0){
      mainDir = 2; // E
    }
    else{
      mainDir = 3; // W
    }
  }
  else{ // main direction change in longitude
    if(deltaLon > 0){
      mainDir = 0; // N
    }
    else{
      mainDir = 1; // S
    }
  }

  lastDirection = mainDir;

  return lastDirection;
}


/* 
void loop(){ 
  while(gpsSerial.available()){ // check for gps data 
    if(gps.encode(gpsSerial.read()))// encode gps data 
    {  
      gps.f_get_position(&lat,&lon); // get latitude and longitude 
      // display position 
      
      Serial.print("Position: "); 
      Serial.print("Latitude:"); 
      Serial.print(lat,6); 
      Serial.print(";"); 
      Serial.print("Longitude:"); 
      Serial.println(lon,6);  
      
      Serial.print(lat); 
      Serial.print(" ");  
  
    } 
  }

  String latitude = String(lat,6); 

  String longitude = String(lon,6); 
  Serial.println(latitude+";"+longitude); 
  delay(1000); 
} */
