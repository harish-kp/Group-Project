/************************************************************************
*
* Test of Pmod NAV (Based on Jim Lindblom's program)
*
*************************************************************************
* Description: Pmod_NAV
* All data (accelerometer, gyroscope, magnetometer) are displayed
* In the serial monitor
*
* Material
* 1. Arduino Uno
* 2. Pmod NAV (dowload library
* https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library)
* Licence Beerware
*
* Wiring
* Module<----------> Arduino
* J1 broche 6 3.3V
* J1 broche 5 GND
* J1 broche 4 A5
* J1 broche 2 A4
************************************************************************/
// Call of libraries
#include <Wire.h>
#include<SoftwareSerial.h>
#include <SparkFunLSM9DS1.h>
#include <LPS25HBSensor.h>
#include <Servo.h>
// Déclaration des adresses du module
#define LSM9DS1_M 0x1E
#define LSM9DS1_AG 0x6B
#if defined(ARDUINO_SAM_DUE)

#define DEV_I2C Wire1   //Define which I2C bus is used. Wire1 for the Arduino Due

#define SerialPort Serial

#else

#define DEV_I2C Wire    //Or Wire

#define SerialPort Serial

#endif
LPS25HBSensor  *PressTemp;
Servo motor;
int pos = 0;
LSM9DS1 imu; // Creation of the object imu

// Configuration du module
#define PRINT_CALCULATED
#define PRINT_SPEED 250
static unsigned long lastPrint = 0;

// The earth's magnetic field varies according to its location.
// Add or subtract a constant to get the right value
// of the magnetic field using the following site
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -0.33 // déclinaison (en degrés) pour Paris.
SoftwareSerial BTserial(0,1);
int Data;
float pressure1 = 946.0;
const int pwm = 6; //Enable
const int in_1 = 4; const int in_2 = 5; // Motor pins
const int pwm2 = 2; //Enable
const int in_1_2 = 3; const int in_2_2 = 7; // Motor pins
int state_r, state = 0;
int flag = 0;
void setup(void)
{
 BTserial.begin(9600);
 Serial.begin(9600); // initialization of serial communication
 imu.settings.device.commInterface = IMU_MODE_I2C; // initialization of the module
 imu.settings.device.mAddress = LSM9DS1_M;
 imu.settings.device.agAddress = LSM9DS1_AG;
 if (!imu.begin())
 {
  Serial.println("Probleme de communication avec le LSM9DS1.");
  BTserial.println("Problem in communication with LSM9DS1");
  while (1);
 }
 DEV_I2C.begin();
 PressTemp = new LPS25HBSensor (&DEV_I2C);
 PressTemp->Enable();
 motor.attach(9);
 pinMode(pwm,OUTPUT); 
 pinMode(in_1,OUTPUT); 
 pinMode(in_2,OUTPUT);
 pinMode(pwm2,OUTPUT); 
 pinMode(in_1_2,OUTPUT); 
 pinMode(in_2_2,OUTPUT);
}

void loop()
{
 if(BTserial.available()){
    Data = analogRead(4);
    state_r = Serial.read();
    state = state_r;
    if (Data !='0'){
      if ( imu.gyroAvailable() )
       {
        imu.readGyro(); // acquisition des données du gyroscope
       }
       if ( imu.accelAvailable() )
       {
        imu.readAccel(); //Acquisition of accelerometer data
       }
       if ( imu.magAvailable() )
       {
        imu.readMag(); // Acquisition of the magnetometer
       }
       
       if ((lastPrint + PRINT_SPEED) < millis())
       {
        printGyro(); // Print "G: gx, gy, gz"
        printAccel(); // Print "A: ax, ay, az"
        printMag(); // Print "M: mx, my, mz"
        printAttitude(imu.ax, imu.ay, imu.az,-imu.my, -imu.mx, imu.mz);
        Serial.println();
        BTserial.println();
        lastPrint = millis();
       }
       float pressure, temperature;
       PressTemp->GetPressure(&pressure);
       PressTemp->GetTemperature(&temperature);
       SerialPort.print("| Pres[hPa]: ");
       BTserial.print("| Pres[hPa]: ");
       SerialPort.print(pressure, 2);
       BTserial.print(pressure, 2);
       SerialPort.print(" | Temp[C]: ");
       BTserial.print(" | Temp[C]: ");
       SerialPort.print(temperature, 2);
       BTserial.print(temperature, 2);
       SerialPort.println(" |"); 
       BTserial.println(" |");
//       SerialPort.println(state);
       delay(250);
       if (state == '0') {
        digitalWrite(in_1, LOW); 
        digitalWrite(in_2, LOW);
        digitalWrite(in_1_2, LOW); // set pin 2 on L293D low
        digitalWrite(in_2_2, LOW); // set pin 7 on L293D low
        for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
          // in steps of 1 degree
          motor.write(pos);              // tell servo to go to position in variable 'pos'
          delay(25);                       // waits 15ms for the servo to reach the position
        }
        for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
          motor.write(pos);              // tell servo to go to position in variable 'pos'
          delay(25);                       // waits 15ms for the servo to reach the position
        }
        if(flag == 0){
          Serial.println("Robot is standby");
//          BTSerial.println("Robot in standby");
          flag=1;
        }
    }
       if (abs(PressTemp->GetPressure(&pressure) - pressure1) >= 0 && state == '2'){
          BTserial.println("Robot is moving forward");
          Serial.println("Robot is moving forward");
          digitalWrite(in_1,LOW); 
          digitalWrite(in_2,HIGH); 
          analogWrite(pwm,255);
          digitalWrite(in_1_2,LOW); 
          digitalWrite(in_2_2,HIGH); 
          analogWrite(pwm2,255);
          motor.write(90);
          flag =1;
        }
        else if (abs(PressTemp->GetPressure(&pressure) - pressure1) >= 0 && state == '1'){
          BTserial.println("Robot is moving backward");
          Serial.println("Robot is moving backward");
          digitalWrite(in_1,HIGH); 
          digitalWrite(in_2,LOW); 
          analogWrite(pwm,255);
          digitalWrite(in_1_2,HIGH); 
          digitalWrite(in_2_2,LOW); 
          analogWrite(pwm2,255);
          motor.write(90);
          flag =1;
        } 
        else
        state = state;
    }
    else if (Data == '0'){
      Serial.println("No data received");
      BTserial.println("No data received");
    }
    else{;}    
 }
 delay(1000);
}

void printGyro()
{
 Serial.print("G: ");
#ifdef PRINT_CALCULATED
 Serial.print(imu.calcGyro(imu.gx), 2);
 Serial.print(", ");
 Serial.print(imu.calcGyro(imu.gy), 2);
 Serial.print(", ");
 Serial.print(imu.calcGyro(imu.gz), 2);
 Serial.println(" deg/s");
#elif defined PRINT_RAW
 Serial.print(imu.gx);
 Serial.print(", ");
 Serial.print(imu.gy);
 Serial.print(", ");
 Serial.println(imu.gz);
#endif
}

void printAccel()
{
 Serial.print("A: ");
#ifdef PRINT_CALCULATED
 Serial.print(imu.calcAccel(imu.ax), 2);
 Serial.print(", ");
 Serial.print(imu.calcAccel(imu.ay), 2);
 Serial.print(", ");
 Serial.print(imu.calcAccel(imu.az), 2);
 Serial.println(" g");
#elif defined PRINT_RAW
 Serial.print(imu.ax);
 Serial.print(", ");
 Serial.print(imu.ay);
 Serial.print(", ");
 Serial.println(imu.az);
#endif

}
void printMag()
{
 Serial.print("M: ");
#ifdef PRINT_CALCULATED
 Serial.print(imu.calcMag(imu.mx), 2);
 Serial.print(", ");
 Serial.print(imu.calcMag(imu.my), 2);
 Serial.print(", ");
 Serial.print(imu.calcMag(imu.mz), 2);
 Serial.println(" gauss");
#elif defined PRINT_RAW
 Serial.print(imu.mx);
 Serial.print(", ");
 Serial.print(imu.my);
 Serial.print(", ");
 Serial.println(imu.mz);
#endif
}

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
 float roll = atan2(ay, az);
 float pitch = atan2(-ax, sqrt(ay * ay + az * az));
 float heading;
 if (my == 0)
  heading = (mx < 0) ? PI : 0;
 else
  heading = atan2(mx, my);
  heading -= DECLINATION * PI / 180;
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;
 Serial.print("Pitch, Roll: ");
 Serial.print(pitch, 2);
 Serial.print(", ");
 Serial.println(roll, 2);
 Serial.print("Heading: "); Serial.println(heading, 2);
}
