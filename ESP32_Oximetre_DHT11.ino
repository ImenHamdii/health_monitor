#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <MAX30105.h>
#include "heartRate.h"
// Wi-Fi and ThingSpeak credentials
const char* ssid = "HUAWEI Y9 Prime 2019";  // Your Wi-Fi SSID
const char* password = "imen1234";  // Your Wi-Fi Password

#define channelID 2784013
const char clientID[] = "PDQ4CiUGMiITBCkYCQYGAR0";  // MQTT Client ID
const char mqttUserName[] = "PDQ4CiUGMiITBCkYCQYGAR0"; // MQTT Username
const char mqttPass[] = "JEWIgz0t8x0ARFnpC4x46634";  // MQTT Password

// DHT sensor settings
#define DHTPIN 5             // Define the GPIO pin where DHT11 is connected
#define DHTTYPE DHT11        // Define the DHT type as DHT11

DHT dht(DHTPIN, DHTTYPE);  // Create an instance of the DHT class

// MQTT server settings
const char* server = "mqtt3.thingspeak.com";
int mqttPort = 1883;  // Use 8883 for secure connection

WiFiClient client;
PubSubClient mqttClient(client);

// MAX30102 sensor
MAX30105 particleSensor;

// ---- Variables pour les données ----
const byte RATE_SIZE = 4; 
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
float heartRate = 0.0;   // Fréquence cardiaque (en BPM)
float oxygenLevel = 0.0; // Saturation en oxygène (en %)
// Function to handle messages from MQTT subscription
void mqttSubscriptionCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Connect to WiFi
void connectWifi() {
  Serial.print("Connecting to Wi-Fi...");
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    delay(5000);
    Serial.print(".");
  }
  Serial.println("Connected to Wi-Fi.");
}

// Connect to MQTT broker
void mqttConnect() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect(clientID, mqttUserName, mqttPass)) {
      
      Serial.println("Connected to MQTT");
            Serial.print("IR value: ");
  Serial.print("SpO2: "); Serial.print(oxygenLevel); Serial.print(" %, ");
  Serial.print("Pouls: "); Serial.print(heartRate); Serial.println(" BPM");

    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(mqttClient.state());
      delay(5000);
    }
  }
}

// Publish to ThingSpeak
void mqttPublish(long pubChannelID, String message) {
  String topicString = "channels/" + String(pubChannelID) + "/publish";
  mqttClient.publish(topicString.c_str(), message.c_str());
}

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  connectWifi();

  // Set up MQTT client
  mqttClient.setServer(server, mqttPort);
  mqttClient.setCallback(mqttSubscriptionCallback);

  // Initialize DHT sensor
  dht.begin();

  //setup MAX30102 sensor
 if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); 
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }

  if (!mqttClient.connected()) {
    mqttConnect();
  }
  mqttClient.loop();

  // Read the MAX30102 sensor
long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();
  if (checkForBeat(irValue) == true)
    {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.println(beatAvg);
    // Calcul approximatif (implémentez des algorithmes précis si besoin)
  oxygenLevel = calculateOxygen(redValue, irValue);
  heartRate = calculateHeartRate(irValue);
    // Afficher les résultats dans le moniteur série
  Serial.print("***********SpO2: "); 
  Serial.print(oxygenLevel); Serial.println(" %, ");
  Serial.print("****Pouls: "); 
  Serial.print(heartRate); Serial.println(" BPM");


  if (irValue < 50000)
    Serial.println(" No finger?");

  Serial.println();
  delay(500);

  // Read temperature and humidity from DHT11
  float humidity = dht.readHumidity();
  float tempC = dht.readTemperature();
  float tempF = dht.readTemperature(true);
  
  // Check if the readings are valid
  if (isnan(humidity) || isnan(tempC) || isnan(tempF)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Prepare the message to send
  String message = "field1=" + String(humidity) + "&field2=" + String(tempC) + "&field3=" + String(tempF) + "&field4=" + String(heartRate)+ "&field5=" + String(oxygenLevel);

  // Publish the message to ThingSpeak
  mqttPublish(channelID, message);

  // Delay to avoid flooding the server (e.g., update every 15 seconds)
  delay(15000);  // 15 seconds
}

// ---- Calcul approximatif de l'oxygène ----
float calculateOxygen(long red, long ir) {
  // Algorithme simple pour tester les données (remplacez par un vrai calcul)
  return (red + ir) % 100; 
}

// ---- Calcul approximatif de la fréquence cardiaque ----
float calculateHeartRate(long ir) {
  // Algorithme simple pour tester les données (remplacez par un vrai calcul)
  return ir % 120; 
}
