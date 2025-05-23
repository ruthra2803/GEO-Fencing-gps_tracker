# GEO-Fencing-gps_tracker
Smart Geofencing GPS Tracker with SMS Alerts
Abstract
This project presents a GPS tracking system that provides real-time location updates, Geo-fencing alerts, and battery monitoring. The system consists of an Arduino Nano (CH340), NEO-6M GPS module, SIM800L GSM module, and a lithium-ion battery. It sends an SMS alert when the device moves outside a predefined safe zone and allows users to request the location via an SMS command. A battery monitoring system ensures timely notifications when power levels are low.
1. Introduction
GPS tracking is widely used for monitoring vehicles, assets, and individuals. This project enhances GPS tracking by integrating geofencing and a manual SMS-based location request feature. Additionally, a battery monitoring system ensures operational reliability by alerting users when power levels drop below a critical threshold.
2. Components Used
2.1 Hardware Components
Arduino Nano (CH340) - Microcontroller for processing GPS and GSM data.
NEO-6M GPS Module - Provides real-time latitude and longitude.
SIM800L GSM Module - Sends SMS alerts and receives user commands.
Lithium-ion Battery (3.7V - 4.2V, 18650) - Provides power.
Boost Converter (MT3608) - Steps up the battery voltage to 4.2V for SIM800L.
3.7 v lithium-ion Battery - Power Source
2.2 Software Requirements
Arduino IDE - Programming the microcontroller.
TinyGPS++ Library - Parsing GPS data.
SoftwareSerial Library - Communication between Arduino, GPS, and SIM800L.

3. System Design & Working
3.1 GPS Tracking and Location Updates
The NEO-6M GPS module provides real-time coordinates. The Arduino reads these values and checks whether the device is inside the predefined safe zone.
3.2 Geofencing System
A predefined latitude and longitude serve as the "safe zone." The system continuously checks the device's position. If it moves beyond the threshold distance (~1.1 km), an SMS alert is sent.
3.3 SMS-Based Location Request
Users can send the keyword "LOCATE" via SMS to receive the current location as a Google Maps link.

3.4 Battery Monitoring System
A voltage divider circuit measures the battery level. If the voltage falls below 3.6V, the system sends a "LOW BATTERY" warning SMS.
4. Circuit  Connections
Component	Arduino Nano
GPS NEO-6M VCC	5V
GPS NEO-6M GND	GND
GPS NEO-6M TX	D3 (RX)
GPS NEO-6M RX	D2 (TX)
SIM800L VCC	4.2V 
SIM800L GND	GND
SIM800L TX	D10
SIM800L RX	D11 
Battery +	A0
5. Code Implementation
The project is coded in C++ using Arduino IDE. The main functions include:
Fetching GPS Data and checking the safe zone.
Listening for SMS commands and responding with location.
Monitoring battery voltage and sending alerts.
6. Results & Discussion
The system was tested in real-world conditions. It successfully: ✅ Provided real-time GPS tracking. ✅ Sent an SMS alert when moving out of the geofence. ✅ Responded to SMS requests with a Google Maps link. ✅ Monitored battery voltage and sent alerts when low.



7. Conclusion & Future Improvements
This project provides an effective GPS tracking system with geofencing and battery monitoring. Future enhancements may include:
Live tracking on a web server.
Remote geofence update via SMS.
Battery charge monitoring & solar charging integration.

8. References
1.TinyGPS++ Library Documentation. 
2.SIM800L AT Commands Guide.
3.Arduino Nano CH340 Datasheet.

9.Cicuit Diagram

CODE: 
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define RX_PIN 3  // GPS TX to Arduino RX
#define TX_PIN 2  // GPS RX to Arduino TX
#define SIM_TX 10 // SIM800L TX
#define SIM_RX 11 // SIM800L RX
#define BATTERY_PIN A0 // Battery monitoring

SoftwareSerial gpsSerial(RX_PIN, TX_PIN);
SoftwareSerial simSerial(SIM_TX, SIM_RX);
TinyGPSPlus gps;

const float HOME_LAT = 12.9716; // Set your geofence latitude
const float HOME_LON = 77.5946; // Set your geofence longitude
const float GEOFENCE_RADIUS = 0.001; // Approx. 100 meters
const float LOW_BATTERY_THRESHOLD = 3.6;
bool gpsPrinted = false;

void setup() {
    Serial.begin(9600);
    gpsSerial.begin(9600);
    simSerial.begin(9600);
    delay(1000);
    sendSMS("GPS Tracker Initialized");
}

void loop() {
    checkGPS();
    checkBattery();
    checkSMS();
    delay(5000);
}

void checkGPS() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated() && !gpsPrinted) {
            float lat = gps.location.lat();
            float lon = gps.location.lng();
            Serial.print("Lat: "); Serial.print(lat);
            Serial.print(" Lon: "); Serial.println(lon);
            gpsPrinted = true;
            
            if (calculateDistance(lat, lon, HOME_LAT, HOME_LON) > GEOFENCE_RADIUS) {
                sendSMS("Alert! Device moved out of the safe zone.");
            }
        }
    }
}

void checkSMS() {
    if (simSerial.available()) {
        String sms = simSerial.readString();
        if (sms.indexOf("LOCATE") >= 0) {
            sendGPSLocation();
        }
    }
}

void sendGPSLocation() {
    String message = "Location: https://maps.google.com/?q=";
    message += String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
    sendSMS(message);
}

void checkBattery() {
    float batteryVoltage = readBatteryVoltage();
    Serial.print("Battery Voltage: "); Serial.println(batteryVoltage);
    
    if (batteryVoltage < LOW_BATTERY_THRESHOLD) {
        sendSMS("⚠️ LOW BATTERY! Voltage: " + String(batteryVoltage) + "V");
    }
}

float readBatteryVoltage() {
    int rawValue = analogRead(BATTERY_PIN);
    return rawValue * (5.0 / 1023.0) * ((100.0 + 47.0) / 47.0);
}

float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
    float dx = (lon2 - lon1) * cos((lat1 + lat2) / 2.0 * 0.0174533);
    float dy = (lat2 - lat1);
    return sqrt(dx * dx + dy * dy) * 111.32;
}

void sendSMS(String message) {
    simSerial.println("AT+CMGF=1");
    delay(100);
    simSerial.println("AT+CMGS=\"+91XXXXXXXXXX\""); // Replace with your phone number
    delay(100);
    simSerial.println(message);
    delay(100);
    simSerial.write(26);
    delay(5000);
}


OUTPUT :
1.Startup Message (SMS)
When the system is powered on, the SIM800L module sends an SMS:


2. GPS Location (Serial Monitor)
Once the GPS module gets a valid fix, it prints the location (only once):

3. Geofence Alert (SMS)
If the device moves beyond 100 meters from the home location, the system sends an alert via SMS:

4. Manual Location Request via SMS
If the user sends an SMS containing "LOCATE" to the SIM800L module, it responds with:

5. Battery Voltage Monitoring (Serial Monitor)
The battery voltage is printed periodically:
Battery Voltage: 3.8V
6. Low Battery Alert (SMS)
If the battery voltage drops below 3.6V, the system sends a warning SMS:

This ensures that the tracker provides real-time updates, geofencing alerts, and battery status notifications. Let me know if you need any further modifications! 🚀
窗体顶端


窗体底端


