#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define GPS_TX 4  // GPS TX to Arduino RX
#define GPS_RX 3  // GPS RX to Arduino TX
#define SIM_TX 7  // SIM800L TX to Arduino RX
#define SIM_RX 6  // SIM800L RX to Arduino TX

SoftwareSerial gpsSerial(GPS_TX, GPS_RX);
SoftwareSerial simSerial(SIM_TX, SIM_RX);
TinyGPSPlus gps;

const float HOME_LAT = 10.940191947144285; // Set your geofence latitude
const float HOME_LON = 76.94458224498173; // Set your geofence longitude
const float GEOFENCE_RADIUS = 0.001; // Approx. 100 meters

const float FIXED_LAT = 10.940191947144285;
const float FIXED_LON = 76.94458224498173;

void setup() {
    Serial.begin(9600);
    gpsSerial.begin(9600);
    simSerial.begin(9600);
    delay(1000);

    Serial.println("\n[ SYSTEM INITIALIZED ]");
    Serial.println("GPS Tracker Starting...");
    sendSMS("GPS Tracker Initialized");

    // Set SIM800L to text mode for SMS
    simSerial.println("AT+CMGF=1");
    delay(100);
    simSerial.println("AT+CNMI=1,2,0,0,0"); // Directly receive SMS to serial
    delay(100);
}

void loop() {
    checkGPS();
    checkSMS();
    delay(2000); // Reduced delay for real-time monitoring
}

void checkGPS() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated()) {
            float lat = gps.location.lat();
            float lon = gps.location.lng();
            Serial.print("[GPS] Lat: "); Serial.print(lat);
            Serial.print(" | Lon: "); Serial.println(lon);

            if (calculateDistance(lat, lon, HOME_LAT, HOME_LON) > GEOFENCE_RADIUS) {
                Serial.println("[ALERT] Device moved out of the safe zone!");
                sendSMS("Alert! Device moved out of the safe zone.");
            }
        }
    }
}

void checkSMS() {
    if (simSerial.available()) {
        String sms = simSerial.readString();
        Serial.println("[SMS RECEIVED]: " + sms);

        if (sms.indexOf("LOCATE") >= 0) {
            Serial.println("[COMMAND] 'LOCATE' received. Sending fixed location...");
            sendFixedLocation();
        }
    }
}

void sendFixedLocation() {
    String message = "Location: https://maps.google.com/?q=";
    message += String(FIXED_LAT, 6) + "," + String(FIXED_LON, 6);
    Serial.println("[ GPS LOCATION]: " + message);
    sendSMS(message);
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float dx = (lon2 - lon1) * cos((lat1 + lat2) / 2.0 * 0.0174533);
    float dy = (lat2 - lat1);
    return sqrt(dx * dx + dy * dy) * 111.32;
}

void sendSMS(String message) {
    Serial.println("[SENDING SMS] " + message);
    simSerial.println("AT+CMGF=1");
    delay(100);
    simSerial.println("AT+CMGS=\"+916383490905\""); // Replace with your phone number
    delay(100);
    simSerial.println(message);
    delay(100);
    simSerial.write(26);
    delay(3000);
    Serial.println("[SMS SENT]");
}
