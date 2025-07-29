#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFi.h>    // Required for ESP32 Wi-Fi functionalities
#include <WiFiUdp.h> // Required for UDP communication
#include <ESPmDNS.h> // Required for mDNS (Multicast DNS)

// Define the delay between sensor readings in milliseconds
uint16_t BNO055_SAMPLERATE_DELAY_MS = 300;

// --- WiFi AP Configuration ---
// SSID for the Access Point
const char* ssid = "BNO055_AP";
// Password for the Access Point (minimum 8 characters for WPA2)
const char* password = "password123"; // Please change this to a strong password!

// --- UDP Configuration ---
// The broadcast IP address for the AP's subnet.
// By default, ESP32 AP mode uses 192.168.4.1 as its own IP,
// so 192.168.4.255 is the broadcast address for clients connected to it.
const IPAddress broadcastIP(192, 168, 4, 255);
// The local UDP port to send data from and for clients to listen on.
const unsigned int localUdpPort = 2390;
// Create a WiFiUDP object to handle UDP communication
WiFiUDP Udp;

// --- mDNS Configuration ---
// The hostname for mDNS. You can access the device via "http://esp.local"
// or discover its services using mDNS tools.
const char* mdnsHostname = "esp";

// Create an instance of the BNO055 sensor
// The sensor ID is 55, I2C address is 0x28, and it uses the default Wire library.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  // Initialize serial communication at 115200 baud rate for debugging
  Serial.begin(115200);

  // Wait for the serial port to open (useful for boards like Leonardo/Micro/Due, though less critical for ESP32)
  while (!Serial) delay(10);

  Serial.println("--- BNO055 Sensor Data Stream via UDP over WiFi AP ---");
  Serial.println("");

  /* Initialise the sensor */
  // Attempt to begin communication with the BNO055 sensor
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    // If the sensor is not detected, halt execution and print error
    while (1) {
      delay(1000); // Keep printing the error to serial
    }
  }

  // Set the BNO055 to use its external crystal for better accuracy
  bno.setExtCrystalUse(true);
  
  // Give the sensor some time to initialize after configuration
  delay(1000);

  // --- Configure WiFi as Access Point ---
  Serial.print("Setting up Wi-Fi Access Point (AP)...");
  WiFi.softAP(ssid, password); // Start the AP with the defined SSID and password
  IPAddress apIP = WiFi.softAPIP(); // Get the IP address assigned to the AP
  Serial.print("AP IP address: ");
  Serial.println(apIP);
  Serial.print("Connect to AP: ");
  Serial.print(ssid);
  Serial.print(" with password: ");
  Serial.println(password);

  // --- Initialize UDP ---
  Udp.begin(localUdpPort); // Start listening for UDP packets on the specified local port
  Serial.print("UDP server started on port ");
  Serial.println(localUdpPort);

  // --- Initialize mDNS ---
  if (!MDNS.begin(mdnsHostname)) { // Start mDNS with the defined hostname
    Serial.println("Error setting up MDNS responder!");
    while(1) {
      delay(1000); // If mDNS fails, halt execution and print error
    }
  }
  Serial.print("MDNS responder started. Access via ");
  Serial.print(mdnsHostname);
  Serial.println(".local");
  // Register a service for the UDP stream, making it discoverable via mDNS
  MDNS.addService("bno055", "udp", localUdpPort);
}

void loop(void)
{
  // Declare sensor event objects for various data types
  sensors_event_t orientationData;   // For Euler angles (Orientation)
  sensors_event_t angVelocityData;   // For Gyroscope data (Angular Velocity)
  sensors_event_t linearAccelData;   // For Linear Acceleration data
  sensors_event_t magnetometerData;  // For Magnetometer data
  sensors_event_t accelerometerData; // For Raw Accelerometer data
  sensors_event_t gravityData;       // For Gravity vector data

  // Retrieve sensor data for each type from the BNO055
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

  // Create a String object to hold all the formatted sensor data
  String dataString = "";

  // Append Millis timestamp
  dataString += "ms:" + String(millis());
  dataString += "\t";

  // Append Euler Angles (Orientation)
  // dataString += "ex:" + String(orientationData.orientation.x, 4);
  // dataString += " ey:" + String(orientationData.orientation.y, 4);
  // dataString += " ez:" + String(orientationData.orientation.z, 4);
  // dataString += "\t"; // Tab for separation

  // Append Gyroscope (Angular Velocity)
  dataString += "avgx:" + String(angVelocityData.gyro.x, 4);
  dataString += " avgy:" + String(angVelocityData.gyro.y, 4);
  dataString += " avgz:" + String(angVelocityData.gyro.z, 4);
  dataString += "\t";

  // Append Linear Acceleration
  dataString += "lax:" + String(linearAccelData.acceleration.x, 4);
  dataString += " lay:" + String(linearAccelData.acceleration.y, 4);
  dataString += " laz:" + String(linearAccelData.acceleration.z, 4);
  dataString += "\t";

  // Append Magnetometer
  dataString += "mx:" + String(magnetometerData.magnetic.x, 4);
  dataString += " my:" + String(magnetometerData.magnetic.y, 4);
  dataString += " mz:" + String(magnetometerData.magnetic.z, 4);
  dataString += "\t";

  // Append Raw Accelerometer
  dataString += "rax:" + String(accelerometerData.acceleration.x, 4);
  dataString += " ray:" + String(accelerometerData.acceleration.y, 4);
  dataString += " raz:" + String(accelerometerData.acceleration.z, 4);
  dataString += "\t";

  // Append Gravity Vector
  // dataString += "agx:" + String(gravityData.acceleration.x, 4);
  // dataString += " agy:" + String(gravityData.acceleration.y, 4);
  // dataString += " agz:" + String(gravityData.acceleration.z, 4);
  // dataString += "\t";

  // Append Quaternion data
  dataString += "qw:" + String(quat.w(), 4);
  dataString += " qx:" + String(quat.x(), 4);
  dataString += " qy:" + String(quat.y(), 4);
  dataString += " qz:" + String(quat.z(), 4);
  dataString += "\t";

  // Append Temperature
  // dataString += "temp:" + String(boardTemp);
  // dataString += "\t";

  // Append Calibration Status
  dataString += "calsys:" + String(calsys);
  dataString += " calgyro:" + String(calgyro);
  dataString += " calaccel:" + String(calaccel);
  dataString += " calmag:" + String(calmag);

  // Print the formatted data to Serial Monitor for local debugging
  Serial.println(dataString);

  // --- Send the data via UDP broadcast ---
  // Begin a UDP packet to the broadcast IP and specified port
  Udp.beginPacket(broadcastIP, localUdpPort);
  // Write the formatted data string to the UDP packet
  Udp.print(dataString);
  // End the packet, sending it over the network
  Udp.endPacket();

  // Delay before the next sensor reading and UDP transmission
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
