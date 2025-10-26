#define TRIG_R 23
#define ECHO_R 18
#define TRIG_F 4
#define ECHO_F 2
#define TRIG_L 16
#define ECHO_L 5

// Speed of sound in cm/us
const float soundSpeed = 0.0343;

// --- Struct must be defined before functions ---
struct Distances {
  float left;
  float front;
  float right;
};

// Function to get distance from a single sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout 30ms
  if (duration == 0) return -1; // no echo detected

  float distance = (duration * soundSpeed) / 2.0;
  return distance; // in cm
}

// Function to read all sensors
Distances readAllDistances() {
  Distances d;
  d.left  = getDistance(TRIG_L, ECHO_L);
  d.front = getDistance(TRIG_F, ECHO_F);
  d.right = getDistance(TRIG_R, ECHO_R);
  return d;
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_L, OUTPUT);
  pinMode(TRIG_F, OUTPUT);
  pinMode(TRIG_R, OUTPUT);

  pinMode(ECHO_L, INPUT);
  pinMode(ECHO_F, INPUT);
  pinMode(ECHO_R, INPUT);

  Serial.println("Distance sensors ready!");
}

void loop() {
  Distances d = readAllDistances();
  Serial.print("Left: ");  Serial.print(d.left);  Serial.print(" cm | ");
  Serial.print("Front: "); Serial.print(d.front); Serial.print(" cm | ");
  Serial.print("Right: "); Serial.print(d.right); Serial.println(" cm");
  delay(200);
}
