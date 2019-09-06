#include <Wire.h>
#include<SoftwareSerial.h>
#include <SparkFunLSM9DS1.h>
#include <LPS25HBSensor.h>
#define LSM9DS1_M 0x1E
#define LSM9DS1_AG 0x6B
#if defined(ARDUINO_SAM_DUE)

#define DEV_I2C Wire1
#define SerialPort Serial

#else

#define DEV_I2C Wire
#define SerialPort Serial

#endif
LPS25HBSensor  *PressTemp;

LSM9DS1 imu;
#define PRINT_CALCULATED
#define PRINT_SPEED 250
static unsigned long lastPrint = 0;
#define DECLINATION -0.33
SoftwareSerial Bluetooth(0,1);
void setup(void)
{
 Bluetooth.begin(9600);
 Serial.begin(9600); // initialization of serial communication
 imu.settings.device.commInterface = IMU_MODE_I2C; // initialization of the module
 imu.settings.device.mAddress = LSM9DS1_M;
 imu.settings.device.agAddress = LSM9DS1_AG;
// pinMode(5,INPUT);//temperature sensor connected to analog 5
// analogReference(DEFAULT);
 if (!imu.begin())
 {
  Serial.println("Problem in communicating with LSM9DS1.");
  while (1);
 }
 DEV_I2C.begin();
 PressTemp = new LPS25HBSensor (&DEV_I2C);
 PressTemp->Enable();
}
void loop()
{
 if (Bluetooth.available()){ //wait for data received
 Data=Bluetooth.read();
 if (Data == '1'){
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
  Bluetooth.println();
  lastPrint = millis();
 }
 float pressure, temperature;
 PressTemp->GetPressure(&pressure);
 PressTemp->GetTemperature(&temperature);
 SerialPort.print("| Pres[hPa]: ");
 Bluetooth.print("| Pres[hPa]: ");
 SerialPort.print(pressure, 2);
 Bluetooth.print(pressure, 2);
 SerialPort.print(" | Temp[C]: ");
 Bluetooth.print(" | Temp[C]: ");
 SerialPort.print(temperature, 2);
 Bluetooth.print(temperature, 2);
 SerialPort.println(" |");
 Bluetooth.println(" |");
 }
 else if (Data == '0'){
  Serial.println("No data received");
  Bluetooth.println("No data received");
 }
 else{;}
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
