#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <WiFi.h>      // Required for ESP32 Wi-Fi functionalities
#include <WiFiUdp.h>   // Required for UDP communication
#include <ESPmDNS.h>   // Required for mDNS (Multicast DNS)
#include "SD.h"
#include "SPI.h"
#include "FS.h"


const int SD_CS_PIN = 5;
const int LED_PIN = 2;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

const char* ssid = "BNO055_AP";
const char* password = "password123";
const IPAddress broadcastIP(192, 168, 4, 255);

const unsigned int localUdpPort = 2390;
WiFiUDP Udp;
const char* mdnsHostname = "esp";

// Create an instance of the BNO055 sensor
// The sensor ID is 55, I2C address is 0x28, and it uses the default Wire library.
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// --- Button and Logging State ---
const int BOOT_BUTTON_PIN = 0; // The BOOT button on ESP32 is usually GPIO 0
bool loggingActive = false;
unsigned long lastButtonPressTime = 0;
const unsigned long BUTTON_DEBOUNCE_DELAY = 100; // Milliseconds for debouncing

File dataFile;
char fileName[32]; // Buffer to store the unique filename

// Function to find a unique filename for the SD card
void findUniqueFileName() {
  int fileIndex = 1;
  while (true) {
    sprintf(fileName, "/imu_%d.csv", fileIndex);
    if (!SD.exists(fileName)) {
      Serial.print("Creating new file: ");
      Serial.println(fileName);
      break;
    }
    fileIndex++;
  }
}

// Function to write headers to the CSV file
void writeCsvHeader() {
  if (dataFile) {
    // Added "Timestamp_ms" to the header
    String header = "Timestamp_ms,Euler_X,Euler_Y,Euler_Z,Gyro_X,Gyro_Y,Gyro_Z,LinearAccel_X,LinearAccel_Y,LinearAccel_Z,Magnetometer_X,Magnetometer_Y,Magnetometer_Z,RawAccel_X,RawAccel_Y,RawAccel_Z,Gravity_X,Gravity_Y,Gravity_Z,Quat_W,Quat_X,Quat_Y,Quat_Z,Temperature,Cal_System,Cal_Gyro,Cal_Accel,Cal_Mag\n";
    dataFile.print(header);
    dataFile.flush(); // Flush header immediately
  }
}

void setup(void) {
  // Initialize serial communication at 115200 baud rate for debugging
  Serial.begin(115200);

  // Wait for the serial port to open
  while (!Serial) delay(10);

  Serial.println("--- BNO055 Sensor Data Stream via UDP over WiFi AP & SD Card Logging ---");
  Serial.println("");

  // Configure LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure LED is off initially

  // Configure BOOT button pin
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);

  /* Initialise the sensor */
  // Attempt to begin communication with the BNO055 sensor
  if (!bno.begin()) {
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
    while (1) {
      delay(1000); // If mDNS fails, halt execution and print error
    }
  }
  Serial.print("MDNS responder started. Access via ");
  Serial.print(mdnsHostname);
  Serial.println(".local");
  // Register a service for the UDP stream, making it discoverable via mDNS
  MDNS.addService("bno055", "udp", localUdpPort);

  // --- Initialize SD Card ---
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Card Mount Failed. Check wiring or card!");
    // If SD card fails, halt execution
    while (1) {
      delay(1000);
    }
  }
  Serial.println("SD card initialized.");
}

void loop(void) {
  // Check the state of the BOOT button
  int buttonState = digitalRead(BOOT_BUTTON_PIN);

  // Debounce the button press
  if (buttonState == LOW && (millis() - lastButtonPressTime) > BUTTON_DEBOUNCE_DELAY) {
    lastButtonPressTime = millis();
    loggingActive = !loggingActive; // Toggle logging state

    if (loggingActive) {
      Serial.println("Logging and Streaming: ON");
      digitalWrite(LED_PIN, HIGH); // Turn LED ON

      // Find a unique filename and open the file
      findUniqueFileName();
      dataFile = SD.open(fileName, FILE_WRITE);
      if (dataFile) {
        Serial.print("Successfully opened ");
        Serial.println(fileName);
        writeCsvHeader(); // Write CSV header
      } else {
        Serial.println("Error opening file for writing.");
        loggingActive = false; // Turn off logging if file cannot be opened
        digitalWrite(LED_PIN, LOW); // Turn LED OFF
      }
    } else {
      Serial.println("Logging and Streaming: OFF");
      digitalWrite(LED_PIN, LOW); // Turn LED OFF
      if (dataFile) {
        dataFile.close(); // Close the file when logging stops
        Serial.println("File closed.");
      }
    }
  }

  if (loggingActive) {
    // Get current timestamp
    unsigned long currentMillis = millis();

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

    // Create a String object to hold all the formatted sensor data for UDP
    String udpDataString = "";

    // Create a String object to hold the formatted sensor data for SD card (CSV)
    String sdDataString = "";

    // Append Timestamp
    udpDataString += "time:" + String(currentMillis);
    udpDataString += "\t";
    sdDataString += String(currentMillis);
    sdDataString += ",";

    // Append Euler Angles (Orientation)
    udpDataString += "ex:" + String(orientationData.orientation.x, 4);
    udpDataString += " ey:" + String(orientationData.orientation.y, 4);
    udpDataString += " ez:" + String(orientationData.orientation.z, 4);
    udpDataString += "\t"; // Tab for separation

    sdDataString += String(orientationData.orientation.x, 4);
    sdDataString += ",";
    sdDataString += String(orientationData.orientation.y, 4);
    sdDataString += ",";
    sdDataString += String(orientationData.orientation.z, 4);
    sdDataString += ",";

    // Append Gyroscope (Angular Velocity)
    udpDataString += "avgx:" + String(angVelocityData.gyro.x, 4);
    udpDataString += " avgy:" + String(angVelocityData.gyro.y, 4);
    udpDataString += " avgz:" + String(angVelocityData.gyro.z, 4);
    udpDataString += "\t";

    sdDataString += String(angVelocityData.gyro.x, 4);
    sdDataString += ",";
    sdDataString += String(angVelocityData.gyro.y, 4);
    sdDataString += ",";
    sdDataString += String(angVelocityData.gyro.z, 4);
    sdDataString += ",";

    // Append Linear Acceleration
    udpDataString += "lax:" + String(linearAccelData.acceleration.x, 4);
    udpDataString += " lay:" + String(linearAccelData.acceleration.y, 4);
    udpDataString += " laz:" + String(linearAccelData.acceleration.z, 4);
    udpDataString += "\t";

    sdDataString += String(linearAccelData.acceleration.x, 4);
    sdDataString += ",";
    sdDataString += String(linearAccelData.acceleration.y, 4);
    sdDataString += ",";
    sdDataString += String(linearAccelData.acceleration.z, 4);
    sdDataString += ",";

    // Append Magnetometer
    udpDataString += "mx:" + String(magnetometerData.magnetic.x, 4);
    udpDataString += " my:" + String(magnetometerData.magnetic.y, 4);
    udpDataString += " mz:" + String(magnetometerData.magnetic.z, 4);
    udpDataString += "\t";

    sdDataString += String(magnetometerData.magnetic.x, 4);
    sdDataString += ",";
    sdDataString += String(magnetometerData.magnetic.y, 4);
    sdDataString += ",";
    sdDataString += String(magnetometerData.magnetic.z, 4);
    sdDataString += ",";

    // Append Raw Accelerometer
    udpDataString += "rax:" + String(accelerometerData.acceleration.x, 4);
    udpDataString += " ray:" + String(accelerometerData.acceleration.y, 4);
    udpDataString += " raz:" + String(accelerometerData.acceleration.z, 4);
    udpDataString += "\t";

    sdDataString += String(accelerometerData.acceleration.x, 4);
    sdDataString += ",";
    sdDataString += String(accelerometerData.acceleration.y, 4);
    sdDataString += ",";
    sdDataString += String(accelerometerData.acceleration.z, 4);
    sdDataString += ",";

    // Append Gravity Vector
    udpDataString += "agx:" + String(gravityData.acceleration.x, 4);
    udpDataString += " agy:" + String(gravityData.acceleration.y, 4);
    udpDataString += " agz:" + String(gravityData.acceleration.z, 4);
    udpDataString += "\t";

    sdDataString += String(gravityData.acceleration.x, 4);
    sdDataString += ",";
    sdDataString += String(gravityData.acceleration.y, 4);
    sdDataString += ",";
    sdDataString += String(gravityData.acceleration.z, 4);
    sdDataString += ",";

    // Append Quaternion data
    udpDataString += "qw:" + String(quat.w(), 4);
    udpDataString += " qx:" + String(quat.x(), 4);
    udpDataString += " qy:" + String(quat.y(), 4);
    udpDataString += " qz:" + String(quat.z(), 4);
    udpDataString += "\t";

    sdDataString += String(quat.w(), 4);
    sdDataString += ",";
    sdDataString += String(quat.x(), 4);
    sdDataString += ",";
    sdDataString += String(quat.y(), 4);
    sdDataString += ",";
    sdDataString += String(quat.z(), 4);
    sdDataString += ",";

    // Append Temperature
    udpDataString += "temp:" + String(boardTemp);
    udpDataString += "\t";
    sdDataString += String(boardTemp);
    sdDataString += ",";

    // Append Calibration Status
    udpDataString += "calsys:" + String(calsys);
    udpDataString += " calgyro:" + String(calgyro);
    udpDataString += " calaccel:" + String(calaccel);
    udpDataString += " calmag:" + String(calmag);

    sdDataString += String(calsys);
    sdDataString += ",";
    sdDataString += String(calgyro);
    sdDataString += ",";
    sdDataString += String(calaccel);
    sdDataString += ",";
    sdDataString += String(calmag);
    sdDataString += "\n"; // Newline for CSV

    // Print the formatted data to Serial Monitor for local debugging
    Serial.println(udpDataString);

    // --- Send the data via UDP broadcast ---
    // Begin a UDP packet to the broadcast IP and specified port
    Udp.beginPacket(broadcastIP, localUdpPort);
    // Write the formatted data string to the UDP packet
    Udp.print(udpDataString);
    // End the packet, sending it over the network
    Udp.endPacket();

    // --- Save data to SD card ---
    if (dataFile) {
      dataFile.print(sdDataString);
      dataFile.flush(); // IMPORTANT: Flush data to SD card immediately
      // Serial.print("Saved to SD: "); // Uncomment for debug
      // Serial.print(sdDataString); // Uncomment for debug
    } else {
      Serial.println("Error: SD file not open.");
    }

    // Delay before the next sensor reading and UDP transmission
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}