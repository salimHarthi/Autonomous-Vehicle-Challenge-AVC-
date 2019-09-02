// all the libraries we need
#include <Arduino.h>
#include "GPS_degree.h" // self created
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include "servo_motor_control.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55); //IMU
SoftwareSerial serial_connection(10, 11); //RX=pin 11, TX=pin 10 GPS pins
TinyGPSPlus gps;//This is the GPS object that will pretty much do all the grunt work with the NMEA data


uint8_t clibrat = 0; // calibration check
uint8_t wire_check;// wire communication between arduion microcontrollers I2C
uint8_t wire_check2;
float get_Distance_old;// distance from GPS_degree function
float get_Distance_average; // average distance
uint8_t LED = 36; //LED pin blue
uint8_t LED1 = 37; //LED pin Green
uint8_t LED2 = 38; //LED pin yellow
uint8_t gps_signal = 0; // to determan if the car is conecting to gps
uint8_t next_step = 0; // step/loop check
uint8_t bearing_old;
uint8_t Distance_count = 0; //get average distance for accuracy
uint8_t pints_number;//determin number of points to clear
uint8_t ss; // degree calculation
uint8_t LPWM = 6; // backward motion speed
uint8_t enL = 24; // backward motion enable
uint8_t RPWM = 5; // fwd speed
uint8_t enR = 23; // fwd enable
uint8_t my_speed = 60; //car speed from 0 to 255
const uint8_t chipSelect = 53;// SD card
float latt = 0.0; //to store latitude from GPS
float lngg = 0.0; //to store longitude from GPS
float dg2; //to store the bearing
uint8_t point_shift = 0; // to make the transtion between GPS point
float get_Distance = 10.0; // to store Distance
float heading_degree;
long encoder_distance;// distance travelled
uint8_t pi_comunication;


//////////////////////////////////
/*input you GPS locations
  last point shuld be your start/end pint*/
float input_lat[]={ 40.604484, 40.604532, 40.604571, 40.604608, 40.604612, 40.604613, 40.604613, 40.604599, 40.604530, 40.604476, 40.604414, 40.604402, 40.604398, 40.604392, 40.604403, 40.604484}; 
float input_lng[]={-83.124845,-83.124843,-83.124844,-83.124843,-83.124912,-83.124989,-83.125066,-83.125108,-83.125102,-83.125106,-83.125116,-83.125033,-83.124960,-83.124909,-83.124844,-83.124845};
///////////////////////////////////
//get the heading dgree from IMU function
float degree() 
{
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}


///////////////////////////
// check calibration
uint8_t IMU_Calibration() 
{
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  //Serial.println(gyro);
  //Serial.println(system);
  //Serial.println(mag);
  if (mag == 3 && /*accel==3 &&*/ gyro == 3 && system == 3) {
    digitalWrite(LED, HIGH);
    return 1;
  }
  else {
    digitalWrite(LED, LOW);
    return 0;
  }
}


/////////////////////////////////
//Get acceleration from IMU
float IMU_ACCEL() 
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  float accel = sqrt(pow(euler.x(), 2) + pow(euler.y(), 2) + pow(euler.z(), 2));
  return accel;
}




// function to setup and connect ot GPS
void GPS() 
{
  while (serial_connection.available()) //While there are characters to come from the GPS

  {

    gps.encode(serial_connection.read());//This feeds the serial NMEA data into the library one char at a time

  }

  /*if(gps.location.isUpdated())//This will pretty much be fired all the time anyway but will at least reduce it to only after a package of NMEA data comes in
    {

      Serial.print("lat= ");
      Serial.print(gps.location.lat(),6);
      Serial.print("lng= ");
      Serial.print(gps.location.lng(),6);
      Serial.println();

      } */
}



void setup() {
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  /////////////////////
  //LED Setup
  pinMode(LED, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  //////////////////////////////
  //start IMU
  bno.begin();
  bno.setExtCrystalUse(true);


  ///////////////////////////////
  serial_connection.begin(9600);//This opens up communications to the GPS
  Serial.begin(9600);
  Serial.println("GPS Start");//Just show to the monitor that the sketch has started
  Wire.begin();



  ////////////////////////////
  //motor setup
  digitalWrite(enL, HIGH);
  digitalWrite(enR, HIGH);
  //////////////////////////////

}


void loop() {
  while (next_step == 0) {
   // Serial.println("Zero");
    GPS(); // using gps function to get reading
    if (gps.location.isUpdated()) { //this will update lat and lng whenever it get a new data and it will calculate dg2 if a new data arived
      latt = gps.location.lat(); // get latitude
      lngg = gps.location.lng(); //get longitude

      Wire.requestFrom(8, 6);// check the communication
      if (Wire.available()) {
        wire_check = Wire.read();
        //Serial.println(wire_check);
      }
       //Serial.println(wire_check);
      Wire.requestFrom(5, 6);// check the communication
      if (Wire.available()) {
        wire_check2 = Wire.read();
      }
      

      if (gps.satellites.value() > 2) { //check gps signal
        gps_signal = 1;
        if (gps_signal==1){
          digitalWrite(LED1,HIGH);
          }
          else{digitalWrite(LED1,LOW);}
      }
      //Serial.println(gps.satellites.value());
      pints_number = ((sizeof(input_lat) / sizeof(float)) - 1); //number of pint we are going to go to

    }

    clibrat = IMU_Calibration(); //check imu
    Serial.print("imu");
    Serial.println(clibrat);

  if (wire_check==1){
    digitalWrite(LED2,HIGH);
    }
    if (pints_number > 0 && gps_signal == 1 && wire_check == 1 && clibrat == 1 /*&& wire_check2==0*/) {
      //Serial.print("system is Calibrated");
      next_step = 1;
    }
    //Serial.println(wire_check);
  }


  while (next_step == 1) {
    GPS();
    Serial.println(1);
    if (gps.location.isUpdated()) { //this will update lat and lng whenever it get a new data and it will calculate dg2 if a new data arived
      latt = gps.location.lat(); // get latitude
      lngg = gps.location.lng(); //get longitude
      Distance_count++;
    }
    if (Distance_count > 5) {
      get_Distance_old = calcDistance(latt, lngg, input_lat[point_shift + 1], input_lng[point_shift + 1]); // geting Distance that will be compard with Distance sensor
      get_Distance_average = ((get_Distance_old + calcDistance(input_lat[point_shift], input_lng[point_shift], input_lat[point_shift + 1], input_lng[point_shift + 1])) / 2);
      analogWrite(RPWM, my_speed); //speed control 0 to 255
      digitalWrite(LPWM, LOW);
      heading_degree = getDegrees(latt, lngg, input_lat[point_shift + 1], input_lng[point_shift + 1]);
      next_step = 2;
      Distance_count = 0;
    }

  }


  while (next_step == 2) {
    //Serial.println(2);
    GPS(); // using gps function to get reading
    if (gps.location.isUpdated()) { //this will update lat and lng whenever it get a new data and it will calculate dg2 if a new data arived
      latt = gps.location.lat(); // get latitude
      lngg = gps.location.lng(); //get longitude

      /////////////////////////
      /////////////////////////

      get_Distance_old = calcDistance(latt, lngg, input_lat[point_shift + 1], input_lng[point_shift + 1]);
      //heading_degree = getDegrees(latt, lngg, input_lat[point_shift + 1], input_lng[point_shift + 1]);

      
    }


    ///////////////////////////////////////////////////////////////////
    // if we are less than 4 meter awy from the point the car will go to the next poin
    if (((encoder_distance) > (get_Distance_average)) /*&& (get_Distance_old < 4)*/) 
    {

      point_shift += 1;
      latt = gps.location.lat(); // get latitude
      lngg = gps.location.lng(); //get longitude
      Wire.beginTransmission(5);
      Wire.write('1');
      Wire.endTransmission(5);

      get_Distance_old = calcDistance(latt, lngg, input_lat[point_shift + 1], input_lng[point_shift + 1]);
      get_Distance_average = ((get_Distance_old + calcDistance(input_lat[point_shift], input_lng[point_shift], input_lat[point_shift + 1], input_lng[point_shift + 1])) / 2);
      heading_degree = getDegrees(latt, lngg, input_lat[point_shift + 1], input_lng[point_shift + 1]);
      
      if (point_shift > pints_number) 
      { // if we reach all pints the go back to point 1
        point_shift = 0;
        analogWrite(RPWM, 0); //speed control 0 to 255
        digitalWrite(LPWM, LOW);
        next_step = 4;
      }
    }



    //////////////////////////////////////////////////////////////////////////////////
    // sending the angle to the nano
    ss = servo_control(degree(), heading_degree);
   // Serial.println(degree());
    if (!(81 < ss < 99)) {
    analogWrite(RPWM, my_speed);
    }
    else  {
    analogWrite(RPWM, 100);

   }

    Wire.beginTransmission(8);
    Wire.write(ss);
    Wire.endTransmission(8);



    //////////////////////////////
    if (IMU_ACCEL() < 1) {
      analogWrite(RPWM, 135);
    }
    ////////////////////////////////
    //get distancefrom encoder
    Wire.requestFrom(5, 6);
    if (Wire.available()) {
      encoder_distance = Wire.read();
    }

    //////////////////////////////////////////
    if (Serial.available()) {
      pi_comunication = Serial.read();
      if ( pi_comunication == '2') {
        next_step = 2;
      }
      else if ( pi_comunication == '3') {
        next_step = 3;
      }
    }

  }

// when the pi control the car update the points
while (next_step == 3) 
{
    if (Serial.available()) {
      pi_comunication = Serial.read();
      if ( pi_comunication == '2') {
        next_step = 2;
      }
      else if ( pi_comunication == '3') {
        next_step = 3;
      }
    }
    GPS(); // using gps function to get reading
    if (gps.location.isUpdated()) { //this will update lat and lng whenever it get a new data and it will calculate dg2 if a new data arived
      latt = gps.location.lat(); // get latitude
      lngg = gps.location.lng(); //get longitude
      get_Distance_old = calcDistance(latt, lngg, input_lat[point_shift + 1], input_lng[point_shift + 1]);
      heading_degree = getDegrees(latt, lngg, input_lat[point_shift + 1], input_lng[point_shift + 1]);
    }

    Wire.requestFrom(5, 6);
    if (Wire.available()) {
      encoder_distance = Wire.read();
    }
if (((encoder_distance) > (get_Distance_average)) /*&& (get_Distance_old < 4)*/) {
      latt = gps.location.lat(); // get latitude
      lngg = gps.location.lng(); //get longitude      
      point_shift += 1;

      Wire.beginTransmission(5);
      Wire.write('1');
      Wire.endTransmission(5);

      get_Distance_old = calcDistance(latt, lngg, input_lat[point_shift + 1], input_lng[point_shift + 1]);
      get_Distance_average = ((get_Distance_old + calcDistance(input_lat[point_shift], input_lng[point_shift], input_lat[point_shift + 1], input_lng[point_shift + 1])) / 2);
      heading_degree = getDegrees(latt, lngg, input_lat[point_shift + 1], input_lng[point_shift + 1]);
  }
}
}
