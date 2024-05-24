#include <SoftwareSerial.h>
#include <Servo.h>

// Servo motor nesnesi
Servo myservo;

// Hedef konum ve yükseklik
const float TARGET_LAT = 47.397742;
const float TARGET_LON = 8.545593;
const float R = 6371000; // Dünya yarıçapı (metre cinsinden)
const float earthG = 9.81;//Dünya'nın yer çekimi ivmesi (m/s^2)
float dummyLat;
float dummyLon;
float dummyAlt = 10;

float dummyAirspeed;
float dummyGroundspeed;
float dummyHeading = 90;

bool moduleReleased = false;

float calculateGreatCircleDistance(float lat1, float lon1, float lat2, float lon2);
void handlePosition(float latitude, float longitude, float altitude, float distance, float averageSpeed);
void handleVFRHUD(float airspeed, float groundspeed, float heading);
bool shouldReleaseModule(float lat, float lon, float alt, float averageSpeed, float distance);
void releaseModule();

void setup() {
  Serial.begin(9600);  // Arduino'nun kendi seri haberleşmesi
  myservo.attach(9);  // Servo motorun bağlı olduğu pin
  myservo.write(0);   // Servo motoru başlangıç pozisyonuna getir
  randomSeed(analogRead(0));
  dummyLat = random(47395, 47396) / 1000.0;
  dummyLon = 8.545593;
  dummyAirspeed = 10;
  dummyGroundspeed = 10;
}

void loop() {
  if(!moduleReleased){
  float distance = calculateGreatCircleDistance(TARGET_LAT, TARGET_LON, dummyLat, dummyLon);
  float averageSpeed = (dummyAirspeed + dummyGroundspeed) / 2;

  dummyLat += (dummyGroundspeed / R) * (180.0 / PI); // Hızlara göre konumu güncelle

  handlePosition(dummyLat, dummyLon, dummyAlt, distance, averageSpeed);
  handleVFRHUD(dummyAirspeed, dummyGroundspeed, dummyHeading);

  // Simülasyonun devam edebilmesi için kısa bir bekleme süresi
  delay(1000);
  }
}

float calculateGreatCircleDistance(float lat1, float lon1, float lat2, float lon2) {
  float phi1 = lat1 * 3.14 / 180.0;  // Latitude 1'i radyan cinsine dönüştür
  float phi2 = lat2 * 3.14 / 180.0;  // Latitude 2'yi radyan cinsine dönüştür
  float deltaPhi = (lat2 - lat1) * 3.14 / 180.0;  // Latitude farkını radyan cinsine dönüştür
  float deltaLambda = (lon2 - lon1) * 3.14 / 180.0;  // Longitude farkını radyan cinsine dönüştür

  float a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
              cos(phi1) * cos(phi2) *
              sin(deltaLambda / 2) * sin(deltaLambda / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  float distance = R * c;
  return distance;
}

void handlePosition(float latitude, float longitude, float altitude, float distance, float averageSpeed) {
  Serial.print("Lat: ");
  Serial.print(latitude);
  Serial.print(" Lon: ");
  Serial.print(longitude);
  Serial.print(" Alt: ");
  Serial.println(altitude);
  Serial.print(" Dist: ");
  Serial.println(distance);
  Serial.print(" Speed: ");
  Serial.println(averageSpeed);

  if (shouldReleaseModule(latitude, longitude, altitude, averageSpeed, distance)) {
    releaseModule();
  }
}

void handleVFRHUD(float airspeed, float groundspeed, float heading) {
  Serial.print("Airspeed: ");
  Serial.print(airspeed);
  Serial.print(" Groundspeed: ");
  Serial.print(groundspeed);
  Serial.print(" Heading: ");
  Serial.println(heading);
}

bool shouldReleaseModule(float lat, float lon, float alt, float averageSpeed, float distance) {
  float arrivalTime = sqrt((2 * alt) / earthG);
  float distanceToTarget = arrivalTime * averageSpeed;
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
