Task 1: DC Motor Driving with L293D H-Bridge
For this task, I used Tinkercad to simulate a four-wheel robot controlled by an Arduino Uno and an L293D H-bridge motor driver. The goal was to control the robot’s movement in three phases: forward motion, backward motion, and alternating rotations.

1. Components Used
   - Arduino Uno
   - Breadboard
   - L293D motor driver
   - 4 DC motors
   - Power supply (simulated through Arduino in Tinkercad)
   - Jumper wires

2. Wiring & Logic
Each pair of DC motors was connected to a channel on the L293D, allowing me to control direction and speed using digital output pins on the Arduino. I assigned one enable pin per motor pair to control motor power, and two input pins per motor to manage direction:
   - ENA & ENB: Enable pins for left and right motor pairs
   - IN1–IN8: Direction control for all four motors (two inputs per motor)

3. Movement Logic
I structured the robot’s movement in phases, all handled inside the loop():
   - Forward for 30 seconds: All motors spin forward using HIGH/LOW configuration.
   - Backward for 60 seconds: Reversed motor directions using LOW/HIGH.
   - Rotate Left and Right Alternately for 60 seconds: One side’s motors run forward while the other side runs backward for left rotation, and vice versa for right.
Between each phase, I used delay() to control the duration. After the movement sequence, the robot stops permanently by disabling all outputs using stopMotors().

4. Code Summary
The code defined separate functions for each type of movement: moveForward(), moveBackward(), rotateLeft(), rotateRight(), and stopMotors(). This made it easier to manage behavior cleanly and logically within the loop().

void moveForward() { /* Set IN1–IN8 to forward configuration */ }
void moveBackward() { /* Set IN1–IN8 to backward configuration */ }
void rotateLeft() { /* Left wheels backward, right wheels forward */ }
void rotateRight() { /* Right wheels backward, left wheels forward */ }
void stopMotors() { /* Set all IN1–IN8 to LOW */ }

This simulation helped me understand how motor directions can be precisely controlled using H-bridge logic and pin toggling.



Task 2: Obstacle Detection with Ultrasonic Sensor and Servo
Building upon the setup from Task 1, I added a micro servo motor and an ultrasonic distance sensor to give the robot the ability to scan for nearby obstacles. This task simulated autonomous decision-making based on sensor input.

1. Components Added
   - Micro servo motor
   - Ultrasonic distance sensor (shared pin for both trig and echo)
   - Jumper wires

2. Servo Scanning Mechanism
The servo motor was programmed to sweep from 0° to 180°, then back to 0°, mimicking a scanning head. At each 10° interval, the ultrasonic sensor measured the distance directly in front of the robot.

3. Obstacle Detection Logic
During the sweep:
   - The servo stops momentarily at each angle.
   - The ultrasonic sensor sends a sound pulse and listens for its echo.
   - If the distance detected is 10 cm or less, an obstacle is considered to be in the way.
   - Upon detecting an obstacle, the robot would theoretically stop and change direction (represented by a pause and message in the serial monitor).
To simplify the design in Tinkercad, I used a shared pin for both the trigger and echo signals of the ultrasonic sensor. The code switches the pin mode between output (to send) and input (to receive) to simulate proper behavior.

4. Code Behavior
   - Used the pulseIn() function to calculate how long the echo takes to return.
   - Converted the pulse duration into distance using the formula:
  distance = duration * 0.034 / 2
   - Any reading of 0 (i.e., no echo) was treated as a distance of 100 cm, assuming it means "no obstacle."
The servo's sweep continues unless an obstacle is found, in which case the sweep breaks early and simulates a reaction delay.

Outcome
Together, these two tasks gave me a strong understanding of:
   - Controlling multiple DC motors using an L293D driver
   - Writing modular code to handle complex motion sequences
   - Integrating sensors with actuators (servo + ultrasonic)
   - Building simple autonomous behavior using obstacle detection
This full simulation helped replicate how an actual robot might navigate and avoid obstacles in real-world environments.





Full code for Task 1:
// Motor control pins
const int ENA = 1;  // Enable pin for motor pair A (pins 1 on L293D)
const int ENB = 9;  // Enable pin for motor pair B (pin 9 on L293D)

const int IN1 = 2;  // Motor A1
const int IN2 = 3;  // Motor A2
const int IN3 = 4;  // Motor B1
const int IN4 = 5;  // Motor B2
const int IN5 = 6;  // Motor C1
const int IN6 = 7;  // Motor C2
const int IN7 = 8;  // Motor D1
const int IN8 = 10; // Motor D2

void setup() {
  // Set all pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  // Enable both motor pairs
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);
}

void loop() {
  // Move forward (all motors forward)
  moveForward();
  delay(30000); // 30 seconds

  // Move backward (all motors backward)
  moveBackward();
  delay(60000); // 1 minute

  // Rotate left and right alternately for 1 minute
  unsigned long startTime = millis();
  while (millis() - startTime < 60000) {
    rotateLeft();
    delay(2000); // rotate left for 2 seconds
    rotateRight();
    delay(2000); // rotate right for 2 seconds
  }

  // Stop motors at the end
  stopMotors();
  while (true); // stop here
}

void moveForward() {
  // Motor A forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Motor B forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Motor C forward
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);

  // Motor D forward
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
}

void moveBackward() {
  // Motor A backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  // Motor B backward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Motor C backward
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);

  // Motor D backward
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
}

void rotateLeft() {
  // Left motors backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Right motors forward
  digitalWrite(IN5, HIGH);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, HIGH);
  digitalWrite(IN8, LOW);
}

void rotateRight() {
  // Left motors forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Right motors backward
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, HIGH);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW);
  digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW);
  digitalWrite(IN8, LOW);
}




Full code for Task 2:
#include <Servo.h>

Servo myservo;

const int servoPin = 9;
const int trigEchoPin = 7; // single pin for both trigger and echo

long duration;
int distance;

void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin);
  pinMode(trigEchoPin, OUTPUT);
}

int getDistance() {
  // Send trigger pulse
  pinMode(trigEchoPin, OUTPUT);
  digitalWrite(trigEchoPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigEchoPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigEchoPin, LOW);

  // Switch pin to input to listen for echo
  pinMode(trigEchoPin, INPUT);
  duration = pulseIn(trigEchoPin, HIGH, 30000); // timeout 30ms

  // Calculate distance in cm
  int dist = duration * 0.034 / 2;
  if (dist == 0) dist = 100; // no reading means far away
  return dist;
}

void loop() {
  // Sweep servo from 0 to 180 and back
  for (int pos = 0; pos <= 180; pos += 10) {
    myservo.write(pos);
    delay(100);
    distance = getDistance();
    Serial.print("Pos: ");
    Serial.print(pos);
    Serial.print(" Distance: ");
    Serial.println(distance);

    if (distance <= 10) {
      Serial.println("Obstacle detected! Changing direction.");
      // You can add motor control to stop or turn here
      delay(1000); // pause to simulate action
      break;
    }
  }
  for (int pos = 180; pos >= 0; pos -= 10) {
    myservo.write(pos);
    delay(100);
    distance = getDistance();
    Serial.print("Pos: ");
    Serial.print(pos);
    Serial.print(" Distance: ");
    Serial.println(distance);

    if (distance <= 10) {
      Serial.println("Obstacle detected! Changing direction.");
      delay(1000);
      break;
    }
  }
}

