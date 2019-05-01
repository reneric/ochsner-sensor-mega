#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

// Update these with values suitable for your hardware/network.
byte mac[] = { 0xE7, 0xC4, 0x64, 0x2E, 0x27, 0xA4 };
IPAddress ip(192, 168, 2, 105);

/*
 * Analog Readings from sensor:
 *
 * ACTIVE: 0-450
 * PRESENT: 451-900
 * IDLE: > 900
 *
 */

/*
 * These three config variables below can be changed
 */
#define STATE_CHANGE_BUFFER_SECONDS 3 // The buffer period before changing states
#define ACTIVE_MAX_INCHES 24          // The max distance for the ACTIVE state
#define PRESENT_MAX_INCHES 48         // The max distance for the PRESENT state

// The Client ID for connecting to the MQTT Broker
// const char "magicSurfaceClient" = "magicSurfaceClient";

// Presence States
#define ACTIVE_STATE 0
#define PRESENT_STATE 1
#define IDLE_STATE 2

#define NUM_STATIONS 6                // The number of stations

// Station Pins
const int L1_sensorPin = 54;          // The proximity sensor pin for L1 (INPUT)
const int L1_activePin = 22;          // The ACTIVE state pin to trigger the L1 matrix/teensy (OUTPUT)
const int L1_presentPin = 23;         // The PRESENT state pin to trigger the L1 matrix/teensy (OUTPUT)

const int L2_sensorPin = 55;          // The proximity sensor pin for L2 (INPUT)
const int L2_activePin = 24;          // The ACTIVE state pin to trigger the L2 matrix/teensy (OUTPUT)
const int L2_presentPin = 25;         // The PRESENT state pin to trigger the L2 matrix/teensy (OUTPUT)

const int L3_sensorPin = 56;          // The proximity sensor pin for L3 (INPUT)
const int L3_activePin = 26;          // The ACTIVE state pin to trigger the L3 matrix/teensy (OUTPUT)
const int L3_presentPin = 27;         // The PRESENT state pin to trigger the L3 matrix/teensy (OUTPUT)

const int R1_sensorPin = 57;          // The proximity sensor pin for R1 (INPUT)
const int R1_activePin = 28;          // The ACTIVE state pin to trigger the R1 matrix/teensy (OUTPUT)
const int R1_presentPin = 29;         // The PRESENT state pin to trigger the R1 matrix/teensy (OUTPUT)

const int R2_sensorPin = 58;          // The proximity sensor pin for R2 (INPUT)
const int R2_activePin = 30;          // The ACTIVE state pin to trigger the R2 matrix/teensy (OUTPUT)
const int R2_presentPin = 31;         // The PRESENT state pin to trigger the R2 matrix/teensy (OUTPUT)

const int R3_sensorPin = 59;          // The proximity sensor pin for R3 (INPUT)
const int R3_activePin = 32;          // The ACTIVE state pin to trigger the R3 matrix/teensy (OUTPUT)
const int R3_presentPin = 33;         // The PRESENT state pin to trigger the R3 matrix/teensy (OUTPUT)

// Initialize the current state for each station
int L1_currentState;
int L2_currentState;
int L3_currentState;
int R1_currentState;
int R2_currentState;
int R3_currentState;

bool L1_waiting = false;
bool L2_waiting = false;
bool L3_waiting = false;
bool R1_waiting = false;
bool R2_waiting = false;
bool R3_waiting = false;

// Initialize the ethernet library
EthernetClient net;
// Initialize the MQTT library
PubSubClient mqttClient(net);

// The Client ID for connecting to the MQTT Broker
const char* CLIENT_ID = "MS_LR";

// The Topic for mqtt messaging
const char* TOPIC = "command/MS_LR";

const char* mqttServer = "192.168.2.10";
const int mqttPort = 1883;

// Station names, used as MQTT Topics
const char stations[NUM_STATIONS][10] = {"MS_L1", "MS_L2", "MS_L3", "MS_R1", "MS_R2", "MS_R3"};

// Station states, used as MQTT Messages
const char states[3][10] = {"ACTIVE", "PRESENT", "IDLE"};

// Put the current states into an array for indexing
int currentStates[NUM_STATIONS] = {L1_currentState, L2_currentState, L3_currentState, R1_currentState, R2_currentState, R3_currentState};
bool stationWaiting[NUM_STATIONS] = {L1_waiting, L2_waiting, L3_waiting, R1_waiting, R2_waiting, R3_waiting};

// Put the pins into arrays for indexing
const int sensorPins[NUM_STATIONS] = {L1_sensorPin, L2_sensorPin, L3_sensorPin, R1_sensorPin, R2_sensorPin, R3_sensorPin};
const int activePins[NUM_STATIONS] = {L1_activePin, L2_activePin, L3_activePin, R1_activePin, R2_activePin, R3_activePin};
const int presentPins[NUM_STATIONS] = {L1_presentPin, L2_presentPin, L3_presentPin, R1_presentPin, R2_presentPin, R3_presentPin};

long lastReconnectAttempt = 0;

// Reconnect to the MQTT broker when the connection is lost
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect with the client ID
    if (mqttClient.connect(CLIENT_ID)) {
        Serial.println("Connected!");
        // Once connected, publish an announcement...
        mqttClient.publish(CLIENT_ID, "CONNECTED", true);

        // Subscribe to each station topic
        for (int i = 0; i < NUM_STATIONS; i++) {
          mqttClient.publish(stations[i], "CONNECTED SENSOR", true);
          mqttClient.subscribe(stations[i]);
        }
    } else {
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
    }
  }
}

boolean reconnect_non_blocking() {
  if (mqttClient.connect(CLIENT_ID)) {
    Serial.println("Connected!");
    // Once connected, publish an announcement...
    mqttClient.publish(CLIENT_ID, "CONNECTED", true);

    // Subscribe to each station topic
    for (int i = 0; i < NUM_STATIONS; i++) {
      mqttClient.publish(stations[i], "CONNECTED SENSOR", true);
      mqttClient.subscribe(stations[i]);
    }
  } else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
  }
  return mqttClient.connected();
}


void messageReceived(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);  
  Serial.println("] ");
  char payloadArr[length+1];
  
  for (unsigned int i=0;i<length;i++)
  {
    payloadArr[i] = (char)payload[i];
  }
  payloadArr[length] = 0;

  Serial.println(payloadArr);  // null terminated array
}


void setup() {
  // Initialize serial communication:
  Serial.begin(9600);

  // Initialize the ethernet connection
  Ethernet.begin(mac, ip);
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(messageReceived);
  
  // Initialize all pins and set currentStates to IDLE
  for (int i = 0; i < NUM_STATIONS; i++) {
    pinMode(sensorPins[i], INPUT);
    pinMode(activePins[i], OUTPUT);
    pinMode(presentPins[i], OUTPUT);
    currentStates[i] = IDLE_STATE;
  }
  lastReconnectAttempt = 0;
}

void loop() {
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect_non_blocking()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    mqttClient.loop();
  }

  // Run each statin through the state machine
  for (int i = 0; i < NUM_STATIONS; i++) {
    stateMachine(i);
  }
  // Small delay to keep things stable
  delay(10);
}

int lastTempState[NUM_STATIONS];
void stateMachine (int pos) {
  int tempState;
  long analogDistance, inches;
  bool waiting = false;
  volatile static unsigned long lastStateChangeTime = 0;
  
  // The raw (voltage) signal coming from the sensor
  analogDistance = analogRead(sensorPins[pos]);
  inches = analogToInches(analogDistance);

  // Get the current sensor state
  tempState = getState(inches);

  // Only set the lastStateChangeTime on the first loop
  if (tempState != lastTempState[pos] && lastTempState[pos] == currentStates[pos]) {
    lastStateChangeTime = millis();
  }

  stationWaiting[pos] = tempState != currentStates[pos] &&
                        tempState == lastTempState[pos] &&
                        (millis() - lastStateChangeTime) < STATE_CHANGE_BUFFER_SECONDS * 1000;
  waiting = currentStates[pos] == IDLE_STATE && stationWaiting[pos];
  if (waiting) Serial.println(waiting);
  
  /*
   * Only send the state update on the first loop.
   *
   * IF the current (temporary) sensor state is not equal to the actual broadcasted state,
   * AND the current (temporary) state is equal to the last current (temporary) state, (it didn't just change)
   * AND the last time the state changed (when tempState !=  lastTempState) was more than the state change buffer,
   * THEN we can safely change the actual state and broadcast it.
   *
   */
  if (
    tempState != currentStates[pos] &&
    tempState == lastTempState[pos] &&
    millis() - lastStateChangeTime > STATE_CHANGE_BUFFER_SECONDS * 1000
  ) {
#if DEBUG == 1    
    Serial.print("Sensor State Changed: ");
    Serial.println(stations[pos]);
    Serial.print("Last State: ");
    Serial.println(currentStates[pos]);
    Serial.print("New State: ");
    Serial.println(tempState);
#endif
    lastStateChangeTime = millis();
    currentStates[pos] = tempState;
    
    // Publish the message for this station. i.e. client.publish("L1", "ACTIVE")
    mqttClient.publish(stations[pos], states[currentStates[pos]], true);
  }

  switch (currentStates[pos]) {
    case ACTIVE_STATE:
      setActive(pos);
      break;
    case PRESENT_STATE:
      setPresent(pos);
      break;
    default:
      if (waiting) {
        setTripped(pos);
      }
      else {
        setIdle(pos);
      }
      break;
  }
  lastTempState[pos] = tempState;
}

void setActive (int pos) {
  // if (pos == 1) Serial.println("set active");
  digitalWrite(activePins[pos], HIGH);
  digitalWrite(presentPins[pos], LOW);
}

void setPresent (int pos) {
  // if (pos == 1) Serial.println("set present");
  digitalWrite(presentPins[pos], HIGH);
  digitalWrite(activePins[pos], LOW);
}

void setIdle (int pos) {
  // if (pos == 1) Serial.println("set idle");
  digitalWrite(activePins[pos], LOW);
  digitalWrite(presentPins[pos], LOW);
}

void setTripped (int pos) {
  // if (pos == 1) Serial.println("set tripped");
  digitalWrite(activePins[pos], HIGH);
  digitalWrite(presentPins[pos], HIGH);
}

// Map analogInput to inches
long analogToInches(long analogDistance) {
#if DEBUG == 1  
  Serial.println(analogDistance);
#endif
  return map(analogDistance, 50, 1023, 10, 55);
}

int getState(long inches) {
  if (inches >= PRESENT_MAX_INCHES) {
    return IDLE_STATE;
  }
  return inches < ACTIVE_MAX_INCHES ? ACTIVE_STATE : PRESENT_STATE;
}
