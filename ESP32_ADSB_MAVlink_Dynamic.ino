#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
// Include MAVLink v2 libraries
#include <MAVLink.h>

// --- Configuration ---
const char* ssid     = "2.4";
const char* password = "password";

// Dynamic coordinates
double currentLat = 0.0;
double currentLon = 0.0;
bool hasGpsPosition = false; // Prevents API call until we have a location
const char* radius = "40"; 

// Serial 1 config on ESP32 (UART2 pins)
#define TX2_PIN 16
#define RX2_PIN 17
#define MAVLINK_SERIAL Serial2

unsigned long lastTime = 0;
unsigned long timerDelay = 15000; // Query API every 15 seconds

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
}

void loop() {
  // 1. Constantly read incoming MAVLink data to update GPS position
  readMAVLink();

  // 2. Fetch ADSB data on timer, but ONLY if we have a valid GPS fix
  if ((millis() - lastTime) > timerDelay) {
    if (WiFi.status() == WL_CONNECTED) {
      if (hasGpsPosition) {
        fetchAndSendToMAVLink();
      } else {
        Serial.println("[WAITING] Waiting for valid GPS position from Flight Controller...");
      }
    }
    lastTime = millis();
  }
}

// Read incoming serial bytes and parse MAVLink messages
void readMAVLink() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while (MAVLINK_SERIAL.available() > 0) {
    uint8_t c = MAVLINK_SERIAL.read();
    
    // Parse the incoming byte
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle specific messages
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: { // Message ID 33
          mavlink_global_position_int_t pos;
          mavlink_msg_global_position_int_decode(&msg, &pos);
          
          // Ensure we actually have a valid fix (coordinates are not exactly 0)
          if (pos.lat != 0 && pos.lon != 0) {
            currentLat = pos.lat / 10000000.0;
            currentLon = pos.lon / 10000000.0;
            
            if (!hasGpsPosition) {
              Serial.println("[GPS LOCK] Acquired position from Flight Controller!");
              hasGpsPosition = true;
            }
          }
          break;
        }
      }
    }
  }
}

void fetchAndSendToMAVLink() {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  
  // Construct URL dynamically with the freshest GPS coordinates
  String apiUrl = "https://opendata.adsb.fi/api/v3/lat/" + String(currentLat, 6) + 
                  "/lon/" + String(currentLon, 6) + "/dist/" + String(radius);
                  
  Serial.printf("[API] Querying: %s\n", apiUrl.c_str());
  
  if (http.begin(client, apiUrl)) {
    int httpResponseCode = http.GET();
    
    if (httpResponseCode == HTTP_CODE_OK) {
      JsonDocument filter;
      filter["ac"][0]["hex"]       = true;
      filter["ac"][0]["flight"]    = true;
      filter["ac"][0]["lat"]       = true;
      filter["ac"][0]["lon"]       = true;
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
          int32_t latE7 = (int32_t)(ac["lat"].as<double>() * 10000000.0);
          int32_t lonE7 = (int32_t)(ac["lon"].as<double>() * 10000000.0);
          int32_t altMM = (int32_t)(ac["alt_baro"].as<double>() * 0.3048 * 1000.0); 
          uint16_t speedCMS = (uint16_t)(ac["gs"].as<double>() * 51.4444);
          uint16_t headingCDeg = (uint16_t)(ac["track"].as<double>() * 100.0);
          int16_t climbRateCMS = (int16_t)(ac["baro_rate"].as<int>() * 0.508);
          uint16_t squawk = ac["squawk"].is<int>() ? ac["squawk"].as<uint16_t>() : 1200;

          // Skip corrupt data lacking coordinates
          if (latE7 == 0 || lonE7 == 0) continue;

          // 3. Build the MAVLink Payload container
          mavlink_message_t msg;
          uint8_t buf[MAVLINK_MAX_PACKET_LEN];

          mavlink_msg_adsb_vehicle_pack(
            1,                  
            MAV_COMP_ID_ADSB,   
            &msg,
            icaoAddress,        
            latE7,              
            lonE7,              
            ADSB_ALTITUDE_TYPE_PRESSURE_QNH, 
            altMM,              
            headingCDeg,        
            speedCMS,           
            climbRateCMS,       
            callsignBuf,        
            ADSB_EMITTER_TYPE_LIGHT, 
            0,                  
            ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_ALTITUDE | ADSB_FLAGS_VALID_HEADING | ADSB_FLAGS_VALID_VELOCITY | ADSB_FLAGS_VALID_CALLSIGN, 
            squawk              
          );

          // 4. Serialize packet payload into raw bytes and stream over Serial
          uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
          MAVLINK_SERIAL.write(buf, len);

          Serial.printf(" -> Sent Target %s (ICAO: 0x%lX) to ArduPilot\n", callsignBuf, icaoAddress);
          delay(20);
        }
      }
    }
    http.end();
  }
}
