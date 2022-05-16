#include <NMEAGPS.h>
#include <NeoSWSerial.h>
#include <SdFat.h>
#include <LSM303.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Servo.h>
#define UPDATE_TIME 20

Servo myServo;
int servoPin = 6;
LSM303 compass;

SdFat sd;
SdFile file;
const int chipSelect = 10;

float AccX, AccY, AccZ;
float accAngleX, accAngleY, Heading;
float magX, magY, magZ;
float AccErrorX, AccErrorY;
int c=0;

float roll, pitch;
float fXm, fYm, fZm;
float fXm_comp,fYm_comp;
const float alpha = 0.15;

float pos = 0;

Adafruit_BMP085 bmp;

NMEAGPS gps;
gps_fix fix;
NeoSWSerial gpsSerial(8,7);


byte PWM_PIN = 3;
 

bool autopilot;


void setup() {
   pinMode(PWM_PIN, INPUT);
  
  //servo setup
  myServo.attach(6);
  pinMode(servoPin,OUTPUT);
//  digitalWrite(servoPin,HIGH);
//  delayMicroseconds(convertAngleToImp(90));
//  digitalWrite(servoPin,LOW);
//  delay(UPDATE_TIME);
  
  // Set the serial baud rate  
  Serial.begin(9600);

  // Initialise the real time clock
  Wire.begin();

  //Initialise accelerometer
  compass.init();
  compass.enableDefault();

  // Try to communicate with an SD card
  if (sd.begin(chipSelect)) {
    Serial.println("Initialised SD card.");
  } else {
   Serial.println("Failed to initialise SD card.");
  }

  //Initialise BMP sensor
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }

  gpsSerial.begin(9600);

  //table headers
  if (file.open("data.csv", O_CREAT | O_WRITE | O_APPEND)) {

  file.print("date");
  file.print(",");
  file.print("time");
  file.print(",");
  file.print("event");
  file.print(",");
  file.print("lat");
  file.print(",");
  file.print("lon");
  file.print(",");
  file.print("gnd_spd");
  file.print(",");
  file.print("gps_alt");
  file.print(",");
  file.print("baro_alt");
  file.print(",");
  file.print("pitch");
  file.print(",");
  file.print("roll");
  file.print(",");
  file.print("heading");

  file.println();
  file.close();
        
  } else {
    Serial.println("Failed to open the file.");
  }

  
}

float mapper(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {


  //pulseIn times the pulse
  int pwm_value = pulseIn(PWM_PIN, HIGH);

  if((pwm_value >= 1200) && (pwm_value <= 2200)){
  autopilot = true;
  }
  else {
    autopilot = false;
  }
  while (gps.available( gpsSerial )) {
    fix = gps.read();
  }

  compass.read();
  
  AccY = compass.a.x/1600.0;
  AccX = -compass.a.y/1600.0;
  AccZ = compass.a.z/1600.0;

  accAngleX = -((atan2(AccY , sqrt(pow(AccX, 2) + pow(AccZ, 2)))) +  0.0185410480);
  accAngleY = (atan2(AccX , sqrt(pow(AccY, 2) + pow(AccZ, 2))))+ 0.0286564636;

  roll = accAngleX*180/PI;
  pitch = accAngleY*180/PI;

  float Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal;

  Xm_off = compass.m.x*(100000.0/1100.0) - -4194.577665; //X-axis combined bias (Non calibrated data - bias)
  Ym_off = compass.m.y*(100000.0/1100.0) - -23721.450236; //Y-axis combined bias (Default: substracting bias)
  Zm_off = compass.m.z*(100000.0/980.0) - 11504.225307; //Z-axis combined bias

  Xm_cal =  1.091415*Xm_off + -0.005580*Ym_off + 0.017370*Zm_off; //X-axis correction for combined scale factors (Default: positive factors)
  Ym_cal =  -0.005580*Xm_off + 1.107018*Ym_off + -0.015021*Zm_off; //Y-axis correction for combined scale factors
  Zm_cal =  0.017370*Xm_off + -0.015021*Ym_off + 1.005984*Zm_off; //Z-axis correction for combined scale factors

  fXm_comp = Xm_cal*cos(pitch/(180/PI)) + Zm_cal*sin(pitch/(180/PI));
  fYm_comp = Xm_cal*sin(roll/(180/PI))*sin(pitch/(180/PI)) + Ym_cal*cos(roll/(180/PI)) - Zm_cal*sin(roll/(180/PI))*cos(pitch/(180/PI));
  Heading = ((atan2(-fYm_comp, fXm_comp))*180/PI) -90;

  if (Heading < 0) {
    Heading = 360 + Heading;
  }

  if (((roll) > 2) && ((roll) <= 22)) {
    //pos = -mapper(roll, 1, 20, -1, -20);
    pos = 1.5*(roll - 2);
  }
  else if (((roll) < -2) && ((roll) >= -22)) {
    //pos = -mapper(roll, -1, -20, 1, 20);
    pos = 1.5*(roll + 2);
  }
  else if (roll > 22) {
    pos = 30;
  }
  else if (roll < -22) {
    pos = -30;
  }
  else if (((roll) <= 2) && ((roll) >= -2)) {
    pos = 0;
  }

  int Angle = pos+90;
//  digitalWrite(servoPin,HIGH);
//  delayMicroseconds(convertAngleToImp(Angle));
//  digitalWrite(servoPin,LOW);
//  delay(UPDATE_TIME);
  myServo.write(Angle);
  Serial.print(pos);  

  Serial.println();
  
  //Data logging
  
    if (file.open("data.csv", O_CREAT | O_WRITE | O_APPEND)) {

    // Store a number 
    file.print(fix.dateTime.date);
    file.print('-');
    file.print(fix.dateTime.month);
    file.print('-');
    file.print(fix.dateTime.year);
    Serial.print(fix.dateTime.date);
    Serial.print('-');
    Serial.print(fix.dateTime.month);
    Serial.print('-');
    Serial.print(fix.dateTime.year);

    // Insert a comma separation
    file.print(",");
    Serial.print(",");

    // Store a number 
    file.print(fix.dateTime.hours);
    file.print('-');
    file.print(fix.dateTime.minutes);
    file.print('-');
    file.print(fix.dateTime.seconds);
    file.print('-');
    file.print(fix.dateTime_ms());
    Serial.print(fix.dateTime.hours);
    Serial.print('-');
    Serial.print(fix.dateTime.minutes);
    Serial.print('-');
    Serial.print(fix.dateTime.seconds); 
    Serial.print('-');
    Serial.print(fix.dateTime_ms());

    // Insert a comma separation
    file.print(",");
    Serial.print(",");

    if(autopilot){
    file.print("Roll Stab ON");
    Serial.print("Roll Stab ON");
    }
    else {
    file.print(' ');
    Serial.print(' ');
    }

    // Insert a comma separation
    file.print(",");
    Serial.print(",");

    // Store a number 
    file.print(fix.latitude());
    Serial.print(fix.latitude());


    // Insert a comma separation
    file.print(",");
    Serial.print(",");

    // Store a number 
    file.print(fix.longitude(),6);
    Serial.print(fix.longitude(),6);

    // Insert a comma separation
    file.print(",");
    Serial.print(",");

    file.print(fix.speed_kph()/3.6);
    Serial.print(fix.speed_kph()/3.6);

    // Insert a comma separation
    file.print(",");
    Serial.print(",");

    file.print(fix.altitude(),6);
    Serial.print(fix.altitude(),6);

    // Insert a comma separation
    file.print(",");
    Serial.print(",");

    // Store a number 
    file.print(bmp.readAltitude());
    Serial.print(bmp.readAltitude());

    // Insert a comma separation
    file.print(",");
    Serial.print(",");

    // Store a number 
    file.print(roll);
    Serial.print(roll);

    // Insert a comma separation
    file.print(",");
    Serial.print(",");
    
    // Store another number 
    file.print(pitch);
    Serial.print(pitch);

    // Insert a comma separation
    file.print(",");
    Serial.print(",");

    // Store another number 
    file.print(Heading);
    Serial.print(Heading);
    
    // finish
    file.println();
    file.close();
    Serial.println();
  } else {
    Serial.println("Failed to open the file.");
  }

  // How long until the next measurement?
}

int convertAngleToImp(int ang){
  float a = 2000/180;
  float b = 500;
  return int(a*ang+b);
}
