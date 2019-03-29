#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

// Update these with values suitable for your hardware/network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 97);
IPAddress server(192, 168, 1, 77);

// ACTIVE: 0-450
// PRESENT: 451-900
// IDLE: > 900

#define ACTIVE_STATE 0
#define PRESENT_STATE 1
#define IDLE_STATE 2

#define STATE_CHANGE_BUFFER_SECONDS 1
#define ACTIVE_MAX_INCHES 24
#define PRESENT_MAX_INCHES 48

// Station Pins
const int L1_sensorPin = A0;
const int L1_activePin = 22;
const int L1_presentPin = 23;

const int L2_sensorPin = A1;
const int L2_activePin = 24;
const int L2_presentPin = 25;

const int L3_sensorPin = A2;
const int L3_activePin = 26;
const int L3_presentPin = 27;

const int R1_sensorPin = A3;
const int R1_activePin = 28;
const int R1_presentPin = 29;

const int R2_sensorPin = A4;
const int R2_activePin = 30;
const int R2_presentPin = 31;

const int R3_sensorPin = A5;
const int R3_activePin = 32;
const int R3_presentPin = 33;

int L1_currentState;
int L2_currentState;
int L3_currentState;
int R1_currentState;
int R2_currentState;
int R3_currentState;


EthernetClient net;
PubSubClient client(net);

// String brokerIp = "192.168.1.77";
const char* mqtt_server = "broker.mqtt-dashboard.co";

const String stations[6] = {"L1", "L2", "L3", "R1", "R2", "R3"};
const String states[3] = {"ACTIVE", "PRESENT", "IDLE"};
int currentStates[6] = {L1_currentState, L2_currentState, L3_currentState, R1_currentState, R2_currentState, R3_currentState};

const int sensorPins[6] = {L1_sensorPin, L2_sensorPin, L3_sensorPin, R1_sensorPin, R2_sensorPin, R3_sensorPin};
const int activePins[6] = {L1_activePin, L2_activePin, L3_activePin, R1_activePin, R2_activePin, R3_activePin};
const int presentPins[6] = {L1_presentPin, L2_presentPin, L3_presentPin, R1_presentPin, R2_presentPin, R3_presentPin};

void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

void reconnect() {
  Serial.print("connecting...");
  while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("arduino")) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            client.publish("magicSurface", "hello world");
            client.subscribe("magicSurface");
            // client.unsubscribe("magicSurface");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }

  Serial.println("\nconnected!");

  
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

unsigned long lastMqttMillis = 0;

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  Ethernet.begin(mac, ip);
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // client.onMessage(messageReceived);
  
  // Init all pins and set currentStates to IDLE
  for (int i = 0; i < 6; i++) {
    pinMode(sensorPins[i], INPUT);
    pinMode(activePins[i], OUTPUT);
    pinMode(presentPins[i], OUTPUT);
    currentStates[i] = IDLE_STATE;
  }
}

void loop() {
  if (!client.connected()) {
        reconnect();
    }
    client.loop();

  for (int i = 0; i < 6; i++) {
    stateMachine(i);
  }

  // publish a message roughly every second.
  //  if (millis() - lastMqttMillis > 1000) {
  //    lastMqttMillis = millis();
  //    client.publish("/hello", "world");
  //  }
  
  delay(100);
}

void stateMachine (int pos) {
  int tempState;
  long analogDistance, inches;
  volatile static int lastTempState = IDLE_STATE;
  volatile static unsigned long lastStateChangeTime = 0;
  unsigned long stateTime = millis();
  
  analogDistance = analogRead(sensorPins[pos]);
  inches = analogToInches(analogDistance);
  
  tempState = getState(inches);

  if (tempState != lastTempState && lastTempState == currentStates[pos]) {
    lastStateChangeTime = millis();
  }
  
  if (tempState != currentStates[pos] && stateTime - lastStateChangeTime > STATE_CHANGE_BUFFER_SECONDS * 1000) {
    lastStateChangeTime = stateTime;
    currentStates[pos] = tempState;
    client.publish(stations[pos], states[pos], 1883);
  }
  if (currentStates[pos] == IDLE_STATE) {
    Serial.println("" + stations[pos] + " IDLE");
    setIdle(pos);
  }
  if (currentStates[pos] == PRESENT_STATE) {
    Serial.println("" + stations[pos] + " PRESENT");
    setPresent(pos);
  }
  if (currentStates[pos] == ACTIVE_STATE) {
    Serial.println("" + stations[pos] + " ACTIVE");
    setActive(pos);
  }
  lastTempState = tempState;
}

void setActive (int pos) {
  digitalWrite(activePins[pos], HIGH);
  digitalWrite(presentPins[pos], LOW);
}

void setPresent (int pos) {
  digitalWrite(presentPins[pos], HIGH);
  digitalWrite(activePins[pos], LOW);
}

void setIdle (int pos) {
  digitalWrite(activePins[pos], LOW);
  digitalWrite(presentPins[pos], LOW);
}

// Map analogInput to inches
long analogToInches(long analogDistance) {
#if DEBUG == 1  
  Serial.println(analogDistance);
#endif
  return map(analogDistance, 50, 1023, 10, 48);
}

int getState(long inches) {
  if (inches >= PRESENT_MAX_INCHES) {
    return IDLE_STATE;
  }
  return inches < ACTIVE_MAX_INCHES ? ACTIVE_STATE : PRESENT_STATE;
}
