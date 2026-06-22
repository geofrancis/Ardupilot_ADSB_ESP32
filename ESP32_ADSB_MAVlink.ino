#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
// Include MAVLink v2 libraries
#include <MAVLink.h>

// --- Configuration ---
const char* ssid     = "2.4";
const char* password = "password";

// Your local coordinates (Example: Helsinki Airport)
const char* latitude  = "50.865";
const char* longitude = "4.433";
const char* radius    = "40"; 

// Serial 1 config on ESP32 (UART2 pins)
#define TX2_PIN 16
#define RX2_PIN 17
#define MAVLINK_SERIAL Serial2

unsigned long lastTime = 0;
unsigned long timerDelay = 15000; // Query API every 15 seconds

String apiUrl = "https://opendata.adsb.fi/api/v3/lat/" + String(latitude) + 
                "/lon/" + String(longitude) + "/dist/" + String(radius);

void setup() {
  Serial.begin(115200); // Debug terminal
  
  // Initialize MAVLink Hardware Serial to Flight Controller
  MAVLINK_SERIAL.begin(57600, SERIAL_8N1, RX2_PIN, TX2_PIN);
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
  
  fetchAndSendToMAVLink();
}

void loop() {
  if ((millis() - lastTime) > timerDelay) {
    if (WiFi.status() == WL_CONNECTED) {
      fetchAndSendToMAVLink();
    }
    lastTime = millis();
  }
}

void fetchAndSendToMAVLink() {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  
  if (http.begin(client, apiUrl)) {
    int httpResponseCode = http.GET();
    
    if (httpResponseCode == HTTP_CODE_OK) {
      JsonDocument filter;
      filter["ac"][0]["hex"]       = true;
      filter["ac"][0]["flight"]    = true;
      filter["ac"][0]["lat"]      = true;
      filter["ac"][0]["lon"]      = true;
      filter["ac"][0]["alt_baro"]  = true;
      filter["ac"][0]["gs"]        = true;
      filter["ac"][0]["track"]     = true;
      filter["ac"][0]["baro_rate"] = true;
      filter["ac"][0]["squawk"]    = true;

      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));
      
      if (!error) {
        JsonArray aircraftList = doc["ac"].as<JsonArray>();
        Serial.printf("[ADSB] Found %d planes. Broadcasting MAVLink packets...\n", aircraftList.size());

        for (JsonObject ac : aircraftList) {
          // 1. Extract and map strings securely
          String flight = ac["flight"].as<String>();
          flight.trim();
          char callsignBuf[9];
          memset(callsignBuf, 0, sizeof(callsignBuf));
          strncpy(callsignBuf, flight.c_str(), 8);

          // Convert ICAO Hex string (e.g. "461f8a") to uint32_t integer
          const char* hexStr = ac["hex"].as<const char*>();
          uint32_t icaoAddress = strtoul(hexStr, NULL, 16);

          // 2. Extract telemetry variables with proper metric scaling
          // MAVLink wants Latitude/Longitude in integers (Degrees * 1E7)
          int32_t latE7 = (int32_t)(ac["lat"].as<double>() * 10000000.0);
          int32_t lonE7 = (int32_t)(ac["lon"].as<double>() * 10000000.0);
          
          // MAVLink wants Altitude in millimeters (mm)
          int32_t altMM = (int32_t)(ac["alt_baro"].as<double>() * 0.3048 * 1000.0); 
          
          // MAVLink wants Speed in cm/s (Knots to cm/s is * 51.4444)
          uint16_t speedCMS = (uint16_t)(ac["gs"].as<double>() * 51.4444);
          
          // MAVLink wants Heading/Track in cdeg (Degrees * 100)
          uint16_t headingCDeg = (uint16_t)(ac["track"].as<double>() * 100.0);
          
          // MAVLink wants climb rate in cm/s (Feet/min to cm/s is * 0.508)
          int16_t climbRateCMS = (int16_t)(ac["baro_rate"].as<int>() * 0.508);
          
          uint16_t squawk = ac["squawk"].is<int>() ? ac["squawk"].as<uint16_t>() : 1200;

          // Skip corrupt data lacking coordinates
          if (latE7 == 0 || lonE7 == 0) continue;

         // 3. Build the MAVLink Payload container
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

mavlink_msg_adsb_vehicle_pack(
  1,                  // System ID (1 = Component system ID)
  MAV_COMP_ID_ADSB,   // Component ID (Identifies this device as an ADSB unit)
  &msg,
  icaoAddress,        // ICAO address
  latE7,              // Latitude (deg * 1E7)
  lonE7,              // Longitude (deg * 1E7)
  ADSB_ALTITUDE_TYPE_PRESSURE_QNH, 
  altMM,              // Altitude (mm)
  headingCDeg,        // Heading (centi-degrees)
  speedCMS,           // Ground Speed (cm/s)
  climbRateCMS,       // Vertical speed (cm/s)
  callsignBuf,        // 8-character callsign array
  ADSB_EMITTER_TYPE_LIGHT, 
  0,                  // Time since last communication
  // FIX: Changed VALID_COORDINATE to VALID_COORDS to match library spec
  ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_ALTITUDE | ADSB_FLAGS_VALID_HEADING | ADSB_FLAGS_VALID_VELOCITY | ADSB_FLAGS_VALID_CALLSIGN, 
  squawk              // Squawk code
);

// 4. Serialize packet payload into raw bytes and stream over Serial
uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
MAVLINK_SERIAL.write(buf, len);

// FIX: Changed format specifier from %X to %lX to fix the compiler warning
Serial.printf(" -> Sent Target %s (ICAO: 0x%lX) to ArduPilot\n", callsignBuf, icaoAddress);
delay(20);
        }
      }
    }
    http.end();
  }
}
