
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


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
#define GAS_SENSOR A2
#define GAS_ALERT_THRESHOLD 33  // Seuil en %

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2); // Adresse 0x27, écran 16 colonnes x 2 lignes


unsigned long previousMillis = 0;
unsigned long interval = 2000;
unsigned long previousMillisChat = 0;
unsigned long intervalChat = 100;

bool pumpOverride = false;
bool pumpState = false; // true = ON, false = OFF

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

  lcd.init(); // Initialiser l'écran
  lcd.backlight(); // Activer le rétroéclairage

}

// --- Capteurs ---
void readDHT(float &temperature, float &humidity) {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
}

float readLight() {
  int raw = analogRead(LIGHT_SENSOR);
  return constrain(map(raw, 800, 200, 0, 100), 0, 100);
}


float readSoil() {
  return map(analogRead(SOIL_SENSOR), 1023, 0, 0, 100);
}

float readHeightThym() {
  digitalWrite(TRIG_PIN_THYM, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_THYM, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_THYM, LOW);
  
  long duration = pulseIn(ECHO_PIN_THYM, HIGH);
  float distance = duration * 0.034 / 2;  // distance en cm
  
  return 58 - distance;  // hauteur du thym en cm
}

float readGas() {
  return map(analogRead(GAS_SENSOR), 0, 1023, 0, 100);
}

float readDistanceChat() {
  digitalWrite(TRIG_PIN_CHAT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_CHAT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_CHAT, LOW);
  long duration = pulseIn(ECHO_PIN_CHAT, HIGH, 30000);
  return (duration == 0) ? -1 : duration * 0.034 / 2;
}

void checkChatProximityAndBuzz() {
  float distance = readDistanceChat();
  if (distance > 1 && distance < 50) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void controlPump(float soilPercent) {
  if (!pumpOverride) {
    float soilDryThreshold = 4.0;
    pumpState = (soilPercent < soilDryThreshold);
    digitalWrite(PUMP_PIN, pumpState ? LOW : HIGH);
  }
}

void processSerialCommand() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "pump_on") {
      pumpOverride = true;
      pumpState = true;
      digitalWrite(PUMP_PIN, LOW);
    } else if (command == "pump_off") {
      pumpOverride = true;
      pumpState = false;
      digitalWrite(PUMP_PIN, HIGH);
    } else if (command == "auto") {
      pumpOverride = false;
    }
  }
}

void checkAnomaliesAndLED(float temperature, float soilPercent, float gasPercent, float distanceChat) {
  bool tempAnomaly = (temperature > 35.0 || temperature < 0.0);
  bool soilAnomaly = (soilPercent < 4.0);
  bool gasAnomaly = (gasPercent >= GAS_ALERT_THRESHOLD);
  bool catDetected = (distanceChat > 1 && distanceChat < 50);

  if (tempAnomaly || soilAnomaly || gasAnomaly || catDetected) {
    digitalWrite(LED_PIN, HIGH); // Alerte : allumer la LED
  } else {
    digitalWrite(LED_PIN, LOW);  // Tout est normal
  }
}


void loop() {
  unsigned long currentMillis = millis();
  float temperature, humidity, lightPercent, soilPercent, heightThym, gasPercent, distanceChat;

  processSerialCommand();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    readDHT(temperature, humidity);
    lightPercent = readLight();
    soilPercent = readSoil();
    heightThym = readHeightThym();
    gasPercent = readGas();
    distanceChat = readDistanceChat();

    controlPump(soilPercent);

    checkAnomaliesAndLED(temperature, soilPercent, gasPercent, distanceChat);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(temperature, 1);
    lcd.print(" H:");
    lcd.print(humidity, 0);

    lcd.setCursor(0, 1);
    lcd.print("Sol:");
    lcd.print(soilPercent, 0);
    lcd.print("% G:");
    lcd.print(gasPercent, 0);

    String json = "{\"temp\":" + String(temperature) +
                  ",\"hum\":" + String(humidity) +
                  ",\"soil\":" + String(soilPercent) +
                  ",\"light\":" + String(lightPercent) +
                  ",\"gas\":" + String(gasPercent) +
                  ",\"height\":" + String(heightThym) +
                  ",\"cat\":" + ((distanceChat > 1 && distanceChat < 50) ? "1" : "0") + "}";

    Serial.println(json);
  }

  if (currentMillis - previousMillisChat >= intervalChat) {
    previousMillisChat = currentMillis;
    checkChatProximityAndBuzz();
  }
}
