#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "mavlink/common/mavlink.h"


// --- Configuration ---
const char* ssid     = "2.4";
const char* password = "password";

// Dynamic coordinates
double currentLat = 0.0;
double currentLon = 0.0;
bool hasGpsPosition = false; // Prevents API call until we have a location
const char* radius = "90";

// Serial 1 config on ESP32 (UART2 pins)
#define TX2_PIN 17
#define RX2_PIN 16
#define MAVLINK_SERIAL Serial2

unsigned long lastApiTime = 0;
unsigned long apiTimerDelay = 15000; // Query API every 15 seconds

unsigned long lastHeartbeatTime = 0;
unsigned long heartbeatInterval = 1000; // 1 second heartbeat

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

  // 2. Send periodic MAVLink HEARTBEAT from ADSB component
  if ((millis() - lastHeartbeatTime) >= heartbeatInterval) {
    sendHeartbeat();
    lastHeartbeatTime = millis();
  }

  // 3. Fetch ADSB data on timer, but ONLY if we have a valid GPS fix
  if ((millis() - lastApiTime) > apiTimerDelay) {
    if (WiFi.status() == WL_CONNECTED) {
      if (hasGpsPosition) {
        fetchAndSendToMAVLink();
      } else {
        Serial.println("[WAITING] Waiting for valid GPS position from Flight Controller...");
      }
    } else {
      Serial.println("[WIFI] Not connected");
    }
    lastApiTime = millis();
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

// Send a MAVLink HEARTBEAT from the ADSB component so the FC knows the bridge is alive
void sendHeartbeat() {
  mavlink_message_t hb;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // system id 1, component id MAV_COMP_ID_ADSB
  // type: MAV_TYPE_ONBOARD_CONTROLLER, autopilot: MAV_AUTOPILOT_INVALID
  // base_mode/custom_mode set to 0, system_status MAV_STATE_ACTIVE
  mavlink_msg_heartbeat_pack(
    1,
    MAV_COMP_ID_ADSB,
    &hb,
    MAV_TYPE_ONBOARD_CONTROLLER,
    MAV_AUTOPILOT_INVALID,
    0,
    0,
    MAV_STATE_ACTIVE
  );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &hb);
  MAVLINK_SERIAL.write(buf, len);
  Serial.print(".");
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
      // Filter doc to only required fields to save memory
      StaticJsonDocument<512> filter;
      filter["ac"][0]["hex"]       = true;
      filter["ac"][0]["flight"]    = true;
      filter["ac"][0]["lat"]       = true;
      filter["ac"][0]["lon"]       = true;
      filter["ac"][0]["alt_baro"]  = true;
      filter["ac"][0]["gs"]        = true;
      filter["ac"][0]["track"]     = true;
      filter["ac"][0]["baro_rate"] = true;
      filter["ac"][0]["squawk"]    = true;

      // Main document sized to handle multiple aircraft
      StaticJsonDocument<8192> doc;
      DeserializationError error = deserializeJson(doc, http.getStream(), DeserializationOption::Filter(filter));

      if (!error) {
        JsonArray aircraftList = doc["ac"].as<JsonArray>();
        Serial.printf("[ADSB] Found %d planes. Broadcasting MAVLink packets...\n", aircraftList.size());

        for (JsonObject ac : aircraftList) {
          // Basic null checks
          if (ac.isNull()) continue;
          if (!ac.containsKey("hex") || !ac.containsKey("lat") || !ac.containsKey("lon")) continue;

          // 1. Extract and map strings securely
          String flight = ac["flight"].isNull() ? String("") : ac["flight"].as<String>();
          flight.trim();
          char callsignBuf[9];
          memset(callsignBuf, 0, sizeof(callsignBuf));
          if (flight.length() > 0) {
            strncpy(callsignBuf, flight.c_str(), 8);
          }

          // Convert ICAO Hex string (e.g. "461f8a") to uint32_t integer
          const char* hexStr = ac["hex"].as<const char*>();
          if (hexStr == nullptr) continue;
          uint32_t icaoAddress = (uint32_t)strtoul(hexStr, NULL, 16);

          // 2. Extract telemetry variables with proper metric scaling
          double lat_d = ac["lat"].as<double>();
          double lon_d = ac["lon"].as<double>();
          int32_t latE7 = (int32_t)(lat_d * 10000000.0);
          int32_t lonE7 = (int32_t)(lon_d * 10000000.0);

          // alt_baro is usually feet; convert to meters then to millimeters
          double alt_baro = ac["alt_baro"].isNull() ? 0.0 : ac["alt_baro"].as<double>();
          int32_t altMM = (int32_t)(alt_baro * 0.3048 * 1000.0);

          // ground speed: ADSB 'gs' is knots; convert to cm/s
          double gs = ac["gs"].isNull() ? 0.0 : ac["gs"].as<double>();
          uint16_t speedCMS = (uint16_t)constrain((int)(gs * 51.4444), 0, 65535);

          // track/heading in degrees, convert to centi-degrees
          double track = ac["track"].isNull() ? 0.0 : ac["track"].as<double>();
          uint16_t headingCDeg = (uint16_t)constrain((int)(track * 100.0), 0, 65535);

          // climb rate: try to parse as m/s; convert to cm/s (signed)
          double baro_rate = ac["baro_rate"].isNull() ? 0.0 : ac["baro_rate"].as<double>();
          int16_t climbRateCMS = (int16_t)constrain((int)(baro_rate * 100.0), -32768, 32767);

          // squawk fallback
          uint16_t squawk = 1200;
          if (ac.containsKey("squawk") && !ac["squawk"].isNull()) {
            // squawk may be string or number
            if (ac["squawk"].is<int>()) squawk = ac["squawk"].as<uint16_t>();
            else {
              String s = ac["squawk"].as<String>();
              squawk = (uint16_t)atoi(s.c_str());
            }
          }

          // Skip corrupt data lacking coordinates
          if (latE7 == 0 || lonE7 == 0) continue;

          // 3. Build the MAVLink ADSB_VEHICLE message
          mavlink_message_t msg;
          uint8_t buf[MAVLINK_MAX_PACKET_LEN];

          mavlink_msg_adsb_vehicle_pack(
            1,                  // system id
            MAV_COMP_ID_ADSB,   // component id
            &msg,
            icaoAddress,        // ICAO address
            latE7,              // lat (1e-7)
            lonE7,              // lon (1e-7)
            ADSB_ALTITUDE_TYPE_PRESSURE_QNH, // altitude type
            altMM,              // altitude in mm
            headingCDeg,        // heading in centi-deg
            speedCMS,           // speed in cm/s
            climbRateCMS,       // climb rate in cm/s
            callsignBuf,        // callsign
            ADSB_EMITTER_TYPE_LIGHT, // emitter type
            0,                  // tslc (time since last communication) - set 0
            ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_ALTITUDE | ADSB_FLAGS_VALID_HEADING | ADSB_FLAGS_VALID_VELOCITY | ADSB_FLAGS_VALID_CALLSIGN, // flags
            squawk              // squawk
          );

          // 4. Serialize packet payload into raw bytes and stream over Serial
          uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
          MAVLINK_SERIAL.write(buf, len);

          Serial.printf(" -> Sent Target %s (ICAO: 0x%lX)\n", callsignBuf, (unsigned long)icaoAddress);
          delay(20);
        }
      } else {
        Serial.printf("[JSON] parse error: %s\n", error.c_str());
      }
    } else {
      Serial.printf("[API] HTTP error: %d\n", httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("[API] begin() failed");
  }
}
