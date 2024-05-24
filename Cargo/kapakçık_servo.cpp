#include <SoftwareSerial.h>
#include <Servo.h>
#include <mavlink.h>

// Servo motor nesnesi
Servo myservo;

// Hedef konum ve yükseklik
const float TARGET_LAT = 47.397742;
const float TARGET_LON = 8.545593;
const float R = 6371000; // Dünya yarıçapı (metre cinsinden)
const float earthG = 9.81;//Dünya'nın yer çekimi ivmesi (m/s^2)
float groundspeed;

bool moduleReleased = false;

float calculateGreatCircleDistance(float lat1, float lon1, float lat2, float lon2);
void handleGlobalPosition(mavlink_global_position_int_t position);
void handleVFRHUD(mavlink_vfr_hud_t vfr_hud);
bool shouldReleaseModule(float lat, float lon, float alt, float averageSpeed, float distance);
void releaseModule();

void setup() {
  Serial.begin(9600);  // Arduino'nun kendi seri haberleşmesi
  pixhawkSerial.begin(57600);  // Pixhawk'ın baud hızına uygun olarak
  myservo.attach(9);  // Servo motorun bağlı olduğu pin
  myservo.write(0);   // Servo motoru başlangıç pozisyonuna getir
}

void loop() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (pixhawkSerial.available()) {
    uint8_t c = pixhawkSerial.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          mavlink_global_position_int_t global_position;
          mavlink_msg_global_position_int_decode(&msg, &global_position);
          handleGlobalPosition(global_position);
          break;
        case MAVLINK_MSG_ID_VFR_HUD:
          mavlink_vfr_hud_t vfr_hud;
          mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);
          handleVFRHUD(vfr_hud);
          break;
      }
    }
  }
}

float calculateGreatCircleDistance(float lat1, float lon1, float lat2, float lon2) {
  float phi1 = lat1 * PI / 180.0;  // Latitude 1'i radyan cinsine dönüştür
  float phi2 = lat2 * PI / 180.0;  // Latitude 2'yi radyan cinsine dönüştür
  float deltaPhi = (lat2 - lat1) * PI / 180.0;  // Latitude farkını radyan cinsine dönüştür
  float deltaLambda = (lon2 - lon1) * PI / 180.0;  // Longitude farkını radyan cinsine dönüştür

  float a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
              cos(phi1) * cos(phi2) *
              sin(deltaLambda / 2) * sin(deltaLambda / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  float distance = R * c;
  return distance;
}

void handleGlobalPosition(mavlink_global_position_int_t position) {
  float latitude = position.lat / 1E7;
  float longitude = position.lon / 1E7;
  float altitude = position.alt / 1000.0; // milimetreden metreye dönüştürme
  
  float distance = calculateGreatCircleDistance(TARGET_LAT, TARGET_LON, latitude, longtitude);

  Serial.print("Lat: ");
  Serial.print(latitude, 7); // Hassasiyeti göstermek için 7 ondalık basamak
  Serial.print(" Lon: ");
  Serial.print(longitude, 7); // Hassasiyeti göstermek için 7 ondalık basamak
  Serial.print(" Alt: ");
  Serial.println(altitude);

  if (shouldReleaseModule(latitude, longitude, altitude, , distance)) {
    releaseModule();
  }
}

void handleVFRHUD(mavlink_vfr_hud_t vfr_hud) {
  float airspeed = vfr_hud.airspeed;
  groundspeed = vfr_hud.groundspeed;
  float heading = vfr_hud.heading;
  Serial.print("Airspeed: ");
  Serial.print(airspeed);
  Serial.print(" Groundspeed: ");
  Serial.print(groundspeed);
  Serial.print(" Heading: ");
  Serial.println(heading);
}


bool shouldReleaseModule(float lat, float lon, float alt, float distance) {
  float arrivalTime = sqrt((2 * alt) / earthG);
  float distanceToTarget = arrivalTime * groundspeed;
  Serial.print(" Travel: ");
  Serial.println(distanceToTarget);
  if (!moduleReleased && distanceToTarget >= distance) {
    return true;
  }
  return false;
}

void releaseModule() {
  Serial.println("Module released!");
  myservo.write(90);  // Servo motoru hareket ettir (örneğin, modülü serbest bırakmak için)
  delay(1000);        // Bir süre bekle
  myservo.write(0);   // Servo motoru tekrar başlangıç pozisyonuna getir
  moduleReleased = true;
}
