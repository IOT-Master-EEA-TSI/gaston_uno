#include <DHT.h>
#include <Wire.h>

// --- Broches capteurs environnementaux ---
#define DHTPIN 5
#define DHTTYPE DHT11
#define LIGHT_SENSOR A0
#define SOIL_SENSOR A1
#define LED_PIN 8
#define PUMP_PIN 9
#define BUZZER_PIN 13

// --- Capteurs ultrason ---
#define TRIG_PIN_THYM 6
#define ECHO_PIN_THYM 7
#define TRIG_PIN_CHAT 4
#define ECHO_PIN_CHAT 3

// --- Capteur de gaz Flying Fish (type MQ) ---
#define GAS_SENSOR A5
#define GAS_ALERT_THRESHOLD 33  // Seuil en %

DHT dht(DHTPIN, DHTTYPE);

// --- Gestion des délais non-bloquants ---
unsigned long previousMillis = 0;
unsigned long interval = 2000;

unsigned long previousMillisChat = 0;
unsigned long intervalChat = 100;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(LED_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  pinMode(TRIG_PIN_THYM, OUTPUT);
  pinMode(ECHO_PIN_THYM, INPUT);

  pinMode(TRIG_PIN_CHAT, OUTPUT);
  pinMode(ECHO_PIN_CHAT, INPUT);
  
  dht.begin();
  //Serial.println(F("🌿 Démarrage du système - Capteurs opérationnels"));
}

// --- Capteur DHT11 ---
void readDHT(float &temperature, float &humidity) {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("Erreur DHT11"));
  }
}

// --- Capteur de lumière ---
float readLight() {
  int lightValue = analogRead(LIGHT_SENSOR);
  return map(lightValue, 0, 1023, 0, 100);
}

// --- Capteur humidité sol ---
float readSoil() {
  int soilValue = analogRead(SOIL_SENSOR);
  return map(soilValue, 1023, 0, 0, 100);
}

// --- Capteur ultrason (thym) ---
float readHeightThym() {
  long durationThym = 0;
  float height = 0;
  digitalWrite(TRIG_PIN_THYM, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_THYM, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_THYM, LOW);
  durationThym = pulseIn(ECHO_PIN_THYM, HIGH);

  if (durationThym == 0 || durationThym > 40000) {
    height = -1;
  } else {
    height = durationThym * 0.034 / 2;
  }
  return height;
}

// --- Capteur de gaz (Flying Fish MQ) ---
float readGas() {
  int analogValue = analogRead(GAS_SENSOR);
  return map(analogValue, 0, 1023, 0, 100);
}

// --- Capteur ultrason (chat) ---
float readDistanceChat() {
  long durationChat = 0;
  float distanceChat = 0;
  
  digitalWrite(TRIG_PIN_CHAT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_CHAT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_CHAT, LOW);
  
  durationChat = pulseIn(ECHO_PIN_CHAT, HIGH, 30000);
  
  if (durationChat == 0) {
    return -1;
  } else {
    distanceChat = durationChat * 0.034 / 2;
    return distanceChat;
  }
}

void loop() {
  unsigned long currentMillis = millis();
  float temperature, humidity, lightPercent, soilPercent, heightThym, gasPercent, distanceChat;

  // Lecture des capteurs toutes les 2 secondes
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Lire les capteurs
    readDHT(temperature, humidity);
    lightPercent = readLight();
    soilPercent = readSoil();
    heightThym = readHeightThym();
    gasPercent = readGas();
    distanceChat = readDistanceChat();

    // Créer le JSON avec les données à envoyer
    String json = "{\"temperature\": " + String(temperature) + 
                  ", \"humidity_air\": " + String(humidity) + 
                  ", \"light\": " + String(lightPercent) + 
                  ", \"soil_moisture\": " + String(soilPercent) + 
                  ", \"height_thym\": " + String(heightThym) + 
                  ", \"gas\": " + String(gasPercent) + 
                  ", \"cat_detected\": " + String(distanceChat) + "}";

    // Envoyer le JSON via le port série
    Serial.println(json);
  }

  // Vérification du mouvement du chat toutes les 100ms
  if (currentMillis - previousMillisChat >= intervalChat) {
    previousMillisChat = currentMillis;
    distanceChat = readDistanceChat();

    if (distanceChat > 1 && distanceChat < 50) {
      Serial.print("[ALERTE MOUVEMENT] Mouvement détecté à ");
      Serial.print(distanceChat);
      Serial.println(" cm");
    }
  }
}
