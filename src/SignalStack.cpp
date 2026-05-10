#include <Arduino.h>

#include "PinAssignments.h"
#include "SignalStack.h"

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);
  mag.initialize();
  Serial.println("Testing device connections...");
  Serial.println(mag.testConnection() ? "AK8975 connection successful" : "AK8975 connection failed");
  calibrateMagnetometer();

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STP_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

#if ARDUINO_WIZNET_5500_EVB_PICO
  analogWriteRange(255);
#endif
  // RoveComm Initialization
  Serial.println("RoveComm Initializing...");
  RoveComm.begin(RC_SIGNALSTACKBOARD_IPADDRESS);
  Serial.println("Complete");
}

void loop()
{

  // Always update where the antenna is currently pointing
  currentAzimuth = readCompassAzimuth();

  // Check for new commands from Base Station
  RoveComm.read(packet);
  switch (packet.dataId)
  {
  case RC_SIGNALSTACKBOARD_OPENLOOP_DATA_ID:
    closedLoopActive = false;
    controlMotor(packet.i16data[0]);
    feedWatchdog();
    break;

  case RC_SIGNALSTACKBOARD_SETGPSTARGET_DATA_ID:
  {
    double_t roverLat = packet.ddata[0];
    double_t roverLon = packet.ddata[1];
    double_t baseLat = packet.ddata[2];
    double_t baseLon = packet.ddata[3];

    // Calculate the absolute bearing to the rover
    targetAzimuth = calculateTargetAzimuth(baseLat, baseLon, roverLat, roverLon);
    closedLoopActive = true;
    feedWatchdog();
    break;
  }

  case RC_SIGNALSTACKBOARD_SETANGLETARGET_DATA_ID:
    // If the basestation sends a direct heading to point at
    targetAzimuth = packet.fdata[0];
    closedLoopActive = true;
    feedWatchdog();
    break;
  }

  if (closedLoopActive)
  {
    currentAzimuth = readCompassAzimuth();
    float error = targetAzimuth - currentAzimuth;

    // Shortest path logic
    if (error > 180)
      error -= 360;
    if (error < -180)
      error += 360;

    float absError = abs(error);
    // DEBUG: Print these values to see if the error is actually shrinking!
    Serial_printf("Target: %f | Current: %f | Error: %f\n", targetAzimuth, currentAzimuth, error);

    // 1. DEADZONE: If within 3 degrees, KILL POWER to motor
    if (abs(error) <= 8)
    {
      controlMotor(0);
    }
    else
    {
      float kP = 10.0;
      int16_t motorSpeed = (int16_t)(error * kP);

      controlMotor(motorSpeed);
    }
  }
  // 10Hz Telemetry Heartbeat
  static uint32_t lastTele = 0;
  if (millis() - lastTele > 100)
  {
    // Send back the antenna's current heading so the UI knows where it's pointing
    RoveComm.write(RC_SIGNALSTACKBOARD_COMPASSANGLE_DATA_ID, 1, &currentAzimuth);
    lastTele = millis();
  }
#if ARDUINO_WIZNET_5500_EVB_PICO
  Watchdog.update();
#endif
}

float calculateTargetAzimuth(double baseLat, double baseLon, double roverLat, double roverLon)
{
  double dLon = (roverLon - baseLon) * M_PI / 180.0;
  double rLat1 = baseLat * M_PI / 180.0;
  double rLat2 = roverLat * M_PI / 180.0;

  double y = sin(dLon) * cos(rLat2);
  double x = cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(dLon);
  float az = atan2(y, x) * 180.0 / M_PI;

  return (az < 0) ? (az + 360) : az; // Normalize to 0-360
}

void controlMotor(int16_t speed)
{
  Serial.printf("controlMotor(%d)\n", speed);
  if (speed == 0)
  {
    analogWrite(STP_PIN, 0);
  }
  else
  {
    // 1. Determine direction
    digitalWrite(DIR_PIN, (speed < 0) ? HIGH : LOW);

    // 2. Get absolute value and clamp it to the max speed
    uint16_t limitedSpeed = abs(speed);
    limitedSpeed = constrain(limitedSpeed, 0, MAX_SPEED);

    // 3. Set frequency and start pulses
    analogWriteFrequency(STP_PIN, limitedSpeed);
    analogWrite(STP_PIN, 128); // 50% duty cycle
  }
}

void calibrateMagnetometer()
{
  controlMotor(350);

  int16_t mx, my, mz;
  int16_t minX = 32767, maxX = -32768;
  int16_t minY = 32767, maxY = -32768;
  int16_t minZ = 32767, maxZ = -32768;

  Serial.println("Rotate the sensor 360 degrees now...");

  unsigned long startTime = millis();
  while (millis() - startTime < 75000)
  { // Calibrate for 10 seconds
    mag.setMode(AK8975_MODE_SINGLE);
    delay(20);
    mag.getHeading(&mx, &my, &mz);

    if (mx < minX)
      minX = mx;
    if (mx > maxX)
      maxX = mx;
    if (my < minY)
      minY = my;
    if (my > maxY)
      maxY = my;
    if (mz < minZ)
      minZ = mz;
    if (mz > maxZ)
      maxZ = mz;
  }

  // Calculate midpoints for hard-iron compensation
  offX = (maxX + minX) / 2.0;
  offY = (maxY + minY) / 2.0;
  offZ = (maxZ + minZ) / 2.0;

  Serial.println("Calibration complete.");
}

float readCompassAzimuth()
{
  int16_t mx_raw, my_raw, mz_raw;
  mag.setMode(AK8975_MODE_SINGLE);
  delay(20);
  mag.getHeading(&mx_raw, &my_raw, &mz_raw);

  // Apply Hard-Iron Offsets
  float x_cal = (float)mx_raw - offX;
  float y_cal = (float)my_raw - offY;
  float z_cal = (float)mz_raw - offZ;

  // --- COORDINATE MAPPING ---
  // Antenna direction = Z axis
  // Sideways direction = X axis
  // We use atan2(sideways, forward)
  float angleRad = atan2(-x_cal, -z_cal);

  float D = angleRad * (180.0 / M_PI);

  // Normalize to [0, 360)
  if (D < 0)
    D += 360.0;

  // Serial.print(">");
  // Serial.print("mx:");
  // Serial.print(mx_raw);
  // Serial.print(",");
  // Serial.print("my:");
  // Serial.print(my_raw);
  // Serial.print(",");
  // Serial.print("mz:");
  // Serial.print(mz_raw);
  // Serial.print(",");
  // Serial.print("D:");
  // Serial.print(D);
  // Serial.println(); // Writes \r\n

  return D;
}

void feedWatchdog() { Watchdog.begin(estop, WATCHDOG_TIMEOUT); }

void estop()
{
  if (!watchdogOverride)
  {
    analogWrite(STP_PIN, 0);
    Serial.println("E-STOP ACTIVATED");
  }
}
