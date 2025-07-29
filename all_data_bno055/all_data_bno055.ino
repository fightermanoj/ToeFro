#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Define the delay between sensor readings in milliseconds
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Create an instance of the BNO055 sensor
// The sensor ID is 55, I2C address is 0x28, and it uses the default Wire library.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);

  // Wait for the serial port to open (for Leonardo/Micro/Due)
  while (!Serial) delay(10);  

  Serial.println("BNO055 Comprehensive Sensor Data Output");
  Serial.println("");

  /* Initialise the sensor */
  // Attempt to begin communication with the BNO055 sensor
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // If the sensor is not detected, halt execution
    while (1);
  }

  // Set the BNO055 to use its external crystal for better accuracy
  bno.setExtCrystalUse(true);
  
  // Give the sensor some time to initialize after configuration
  delay(1000);
}


void loop(void)
{
  // Declare sensor event objects for various data types
  sensors_event_t orientationData;  // For Euler angles (Orientation)
  sensors_event_t angVelocityData;  // For Gyroscope data (Angular Velocity)
  sensors_event_t linearAccelData;  // For Linear Acceleration data
  sensors_event_t magnetometerData; // For Magnetometer data
  sensors_event_t accelerometerData; // For Raw Accelerometer data
  sensors_event_t gravityData;      // For Gravity vector data

  // Retrieve sensor data for each type
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  // Retrieve Quaternion data
  imu::Quaternion quat = bno.getQuat();

  // Retrieve board temperature
  int8_t boardTemp = bno.getTemp();

  // Retrieve calibration status for System, Gyroscope, Accelerometer, and Magnetometer
  uint8_t calsys, calgyro, calaccel, calmag = 0;
  bno.getCalibration(&calsys, &calgyro, &calaccel, &calmag);

  // Print all data points on a single line with specified labels

  // Euler Angles (Orientation)
  Serial.print("ex:"); Serial.print(orientationData.orientation.x, 4);
  Serial.print(" ey:"); Serial.print(orientationData.orientation.y, 4);
  Serial.print(" ez:"); Serial.print(orientationData.orientation.z, 4);
  Serial.print("\t");

  // Gyroscope (Angular Velocity)
  Serial.print("avgx:"); Serial.print(angVelocityData.gyro.x, 4);
  Serial.print(" avgy:"); Serial.print(angVelocityData.gyro.y, 4);
  Serial.print(" avgz:"); Serial.print(angVelocityData.gyro.z, 4);
  Serial.print("\t");

  // Linear Acceleration
  Serial.print("lax:"); Serial.print(linearAccelData.acceleration.x, 4);
  Serial.print(" lay:"); Serial.print(linearAccelData.acceleration.y, 4);
  Serial.print(" laz:"); Serial.print(linearAccelData.acceleration.z, 4);
  Serial.print("\t");

  // Magnetometer
  Serial.print("mx:"); Serial.print(magnetometerData.magnetic.x, 4);
  Serial.print(" my:"); Serial.print(magnetometerData.magnetic.y, 4);
  Serial.print(" mz:"); Serial.print(magnetometerData.magnetic.z, 4);
  Serial.print("\t");

  // Raw Accelerometer
  Serial.print("rax:"); Serial.print(accelerometerData.acceleration.x, 4);
  Serial.print(" ray:"); Serial.print(accelerometerData.acceleration.y, 4);
  Serial.print(" raz:"); Serial.print(accelerometerData.acceleration.z, 4);
  Serial.print("\t");

  // Gravity Vector
  Serial.print("agx:"); Serial.print(gravityData.acceleration.x, 4);
  Serial.print(" agy:"); Serial.print(gravityData.acceleration.y, 4);
  Serial.print(" agz:"); Serial.print(gravityData.acceleration.z, 4);
  Serial.print("\t");

  // Quaternion
  Serial.print("qw:"); Serial.print(quat.w(), 4);
  Serial.print(" qx:"); Serial.print(quat.x(), 4);
  Serial.print(" qy:"); Serial.print(quat.y(), 4);
  Serial.print(" qz:"); Serial.print(quat.z(), 4);
  Serial.print("\t");

  // Temperature
  Serial.print("temp:"); Serial.print(boardTemp);
  Serial.print("\t");

  // Calibration Status
  Serial.print("calsys:"); Serial.print(calsys);
  Serial.print(" calgyro:"); Serial.print(calgyro);
  Serial.print(" calaccel:"); Serial.print(calaccel);
  Serial.print(" calmag:"); Serial.print(calmag);
  Serial.println(); // New line after all data points for this iteration

  // Delay before the next reading
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
