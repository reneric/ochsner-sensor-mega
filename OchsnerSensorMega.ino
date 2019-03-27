// ACTIVE: 0-450
// PRESENT: 451-900
// IDLE: > 900

#define ACTIVE_STATE 0
#define PRESENT_STATE 1
#define IDLE_STATE 2

#define STATE_CHANGE_BUFFER_SECONDS 3
#define ACTIVE_MAX_INCHES 24
#define PRESENT_MAX_INCHES 48

const int sensorPin = A0;

const int activePin = 22;
const int presentPin = 23;

int currentState = IDLE_STATE;

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  pinMode(activePin, OUTPUT);
  pinMode(presentPin, OUTPUT);
}

void loop() {
  long analogDistance, inches;
  int tempState;
  volatile static unsigned long lastTempState = IDLE_STATE;
  volatile static unsigned long lastStateChangeTime = 0;
  unsigned long stateTime = millis();
  
  analogDistance = analogRead(sensorPin);
  inches = analogToInches(analogDistance);
//  Serial.print("analogDistance: ");
//  Serial.println(analogDistance);
//  Serial.print("inches: ");
//  Serial.println(inches);
  
  tempState = getState(inches);

  if (tempState != lastTempState && lastTempState == currentState) {
    lastStateChangeTime = millis();
  }
  
  if (tempState != currentState && stateTime - lastStateChangeTime > STATE_CHANGE_BUFFER_SECONDS * 1000) {
    lastStateChangeTime = stateTime;
    currentState = tempState;
    
  }
  if (currentState == IDLE_STATE) {
      Serial.println("IDLE");
      setIdle();
    }
    if (currentState == PRESENT_STATE) {
      Serial.println("PRESENT");
      setPresent();
    }
    if (currentState == ACTIVE_STATE) {
      Serial.println("ACTIVE");
      setActive();
    }
  lastTempState = tempState;
  
  
  
  delay(100);
}

void setActive () {
  digitalWrite(activePin, HIGH);
  digitalWrite(presentPin, LOW);
}

void setPresent () {
  digitalWrite(presentPin, HIGH);
  digitalWrite(activePin, LOW);
}

void setIdle () {
  digitalWrite(activePin, LOW);
  digitalWrite(presentPin, LOW);
}

long stateMachine(long inches) {}

long analogToInches(long analogDistance) {
  return map(analogDistance, 50, 1023, 10, 48);
}

int getState(long inches) {
  if (inches >= PRESENT_MAX_INCHES) {
    return IDLE_STATE;
  }
  return inches < ACTIVE_MAX_INCHES ? ACTIVE_STATE : PRESENT_STATE;
}
