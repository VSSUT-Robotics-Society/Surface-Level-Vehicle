#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <math.h>

#define ch8 12
#define ch2 7
#define ch4 8
int initialFreq = 990, FinalFreq = 1980, Middlepoint = 1490;
unsigned long Switch, Movement, rotation;

#define HMC5883L_ADDR 0x1E
#define HMC5883L_REG_DATA 0x03
// Pin definitions
#define ESC1_PIN 5
#define ESC2_PIN 6

// ESC calibration timing
unsigned long calibrationTime = 2000; // Time to send max signal, in milliseconds
unsigned long minSignalTime = 2000;   // Time to send min signal, in milliseconds
unsigned long maxSignalTime = 2000;   // Time to send max signal, in milliseconds

// PWM signal range (typically between 1000 to 2000 microseconds)
int minPWM = 1000;
int maxPWM = 2000;
int neutralPWM = 1500; // Neutral signal (idle position for most ESCs)
SoftwareSerial serial_connection(2, 3);
TinyGPSPlus gps;
double lat1, lng1, lat2, lng2, magneticDeclination, trueBearing, distance;
int speed, initSpeed = 0, finalSpeed = 255;
double Target[2] = {21.4962951, 83.8977747};
double Current[2] = {0.0, 0.0};
double Kp = 2.0;

#define EARTH_RADIUS 6371.0

// Convert degrees to radians
double toRadians(double degree)
{
  return degree * (M_PI / 180.0);
}

void Calibration()
{
  Serial.println("Starting ESC calibration...");

  // Send max signal to ESCs (2000 microseconds)
  Serial.println("Sending max signal (2000 µs)...");
  analogWrite(ESC1_PIN, map(maxPWM, 0, 255, 0, 255));
  analogWrite(ESC2_PIN, map(maxPWM, 0, 255, 0, 255));
  delay(calibrationTime); // Hold max signal for calibration time

  // Send min signal to ESCs (1000 microseconds)
  Serial.println("Sending min signal (1000 µs)...");
  analogWrite(ESC1_PIN, map(minPWM, 0, 255, 0, 255));
  analogWrite(ESC2_PIN, map(minPWM, 0, 255, 0, 255));
  delay(minSignalTime); // Hold min signal for calibration time

  // Send neutral signal to ESCs (1500 microseconds)
  Serial.println("Sending neutral signal (1500 µs)..");
  analogWrite(ESC1_PIN, map(neutralPWM, 0, 255, 0, 255));
  analogWrite(ESC2_PIN, map(neutralPWM, 0, 255, 0, 255));
  delay(maxSignalTime); // Hold neutral signal for calibration time

  // End of calibration
  Serial.println("ESC calibration complete.");
}

void CurrentLocation()
{
  while (serial_connection.available())
  {
    gps.encode(serial_connection.read());
  }
  if (gps.location.isUpdated())
  {
    Current[0] = gps.location.lat();
    Current[1] = gps.location.lng();
  }
}

double haversineDistance(double lat1, double lon1, double lat2, double lon2)
{
  double dLat = toRadians(lat2 - lat1);
  double dLon = toRadians(lon2 - lon1);

  lat1 = toRadians(lat1);
  lat2 = toRadians(lat2);

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS * c; // Distance in km
}

double initialBearing(double lat1, double lon1, double lat2, double lon2)
{
  lat1 = toRadians(lat1);
  lat2 = toRadians(lat2);
  double dLon = toRadians(lon2 - lon1);

  double y = sin(dLon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

  double bearing = atan2(y, x);
  bearing = bearing * (180.0 / M_PI);    // Convert radians to degrees
  return fmod((bearing + 360.0), 360.0); // Normalize to 0-360 degrees
}

void initCompass()
{
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();
}

void readCompass(int16_t &x, int16_t &y, int16_t &z)
{
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(HMC5883L_REG_DATA);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);
  if (Wire.available() == 6)
  {
    x = (Wire.read() << 8) | Wire.read();
    z = (Wire.read() << 8) | Wire.read();
    y = (Wire.read() << 8) | Wire.read();
  }
}

void turnLeft(int speed)
{
  analogWrite(ESC1_PIN, 0);
  analogWrite(ESC2_PIN, speed);
  Serial.println("Turning Left");
}
void turnRight(int speed)
{
  analogWrite(ESC1_PIN, speed);
  analogWrite(ESC2_PIN, 0);
  Serial.println("Turning Right");
}
void stop()
{
  analogWrite(ESC1_PIN, 0);
  analogWrite(ESC2_PIN, 0);
  Serial.println("Stopped, facing destination");
}

void Forward(int speed)
{
  analogWrite(ESC1_PIN, speed);
  analogWrite(ESC2_PIN, speed);
  Serial.println("Moving Forward");
}

void Angularadjust()
{
  int16_t x, y, z;
  readCompass(x, y, z);

  // Compute magnetic heading
  float magneticHeading = atan2(y, x) * 180.0 / M_PI;
  if (magneticHeading < 0)
    magneticHeading += 360;

  // Convert to true heading
  float trueHeading = magneticHeading + magneticDeclination;
  if (trueHeading > 360)
    trueHeading -= 360;
  if (trueHeading < 0)
    trueHeading += 360;

  // Calculate required rotation
  float angleDifference = trueBearing - trueHeading;
  if (angleDifference > 180)
    angleDifference -= 360;
  if (angleDifference < -180)
    angleDifference += 360;

  Serial.print("True Heading: ");
  Serial.print(trueHeading);
  Serial.print(" Angle Difference: ");
  Serial.println(angleDifference);

  // Apply P-Controller for turning speed
  int turnSpeed = Kp * abs(angleDifference);
  turnSpeed = constrain(turnSpeed, 50, 255);

  if (angleDifference > 5)
  {
    turnLeft(turnSpeed);
  }
  else if (angleDifference < -5)
  {
    turnRight(turnSpeed);
  }
  else
  {
    stop();
  }

  delay(100);
}

void LinearAdjustment()
{
  double distanceToTarget = haversineDistance(Current[0], Current[1], Target[0], Target[1]);
  double distance = distanceToTarget * 1000;
  if (distanceToTarget > 5)
  {                                                  // If more than 0.5 km away
    double error = distanceToTarget - 5;             // Error: distance from the threshold
    speed = Kp * error;                              // P controller formula
    speed = constrain(speed, initSpeed, finalSpeed); // Ensure speed remains within limits
    Forward(speed);
  }
  else
  {
    stop();
  }
}

double getApproximateDeclination(double lat, double lon)
{
  return 0.1 * lat - 0.02 * lon;
}

void Autonavigation()
{
  CurrentLocation();
  lat1 = Current[0];
  lng1 = Current[1];
  lat2 = Target[0];
  lng2 = Target[1];
  magneticDeclination = getApproximateDeclination(lat1, lng1);
  trueBearing = initialBearing(lat1, lng1, lat2, lng2);
  distance = haversineDistance(lat1, lng1, lat2, lng2);
  Angularadjust();
  delay(1000);
  LinearAdjustment();
  // Serial.println(distance,6);
  // Serial.println(magneticDeclination,6);
}

void ManualNavigation(int Movement, int rotation)
{
  if (Movement >= initialFreq - 10 && Movement <= initialFreq + 200)
  {
    Forward(150);
  }
  else if (Movement >= Middlepoint - 10 && Movement <= Middlepoint + 10)
  {
    Forward(0);
  }
  else if (Movement >= FinalFreq - 200 && Movement <= FinalFreq )
  {
    Forward(254);
  }

  if (rotation >= initialFreq  && rotation <= initialFreq + 200)
  {
    turnLeft(254);
    turnRight(0);
  }
  else if (rotation >= Middlepoint - 10 && rotation <= Middlepoint + 10)
  {
    turnLeft(0);
    turnRight(0);
  }
  else if (rotation >= FinalFreq - 200 && rotation <= FinalFreq)
  {
    turnLeft(0);
    turnRight(254);
  }
  else{
    stop();
  }
}

void setup()
{
  Serial.begin(9600);
  serial_connection.begin(9600);
  Serial.println("GPS Start, by akramslab");
  pinMode(ESC1_PIN, OUTPUT);
  pinMode(ESC2_PIN, OUTPUT);
  pinMode(ch8, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch2, INPUT);
  Calibration();
  Wire.begin();
  initCompass();
}

void loop()
{
  Switch = pulseIn(ch8, HIGH, 25000);
  Movement = pulseIn(ch2, HIGH, 25000);
  rotation = pulseIn(ch4, HIGH, 25000);
  if (Switch != 0 && Movement != 0 && rotation != 0)
  {
    if (Switch >= initialFreq - 10 && Switch <= initialFreq + 10)
    {
      ManualNavigation(Movement, rotation);
    }
    else if (Switch >= FinalFreq - 10 && Switch <= FinalFreq + 10)
    {
      Autonavigation();
    }
  }
  else
  {
    Serial.print("RC is not connected");
    stop();
  }
}