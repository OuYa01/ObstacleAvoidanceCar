#include <Servo.h>

//moteur 1
#define enA 11
#define in1 12
#define in2 2

//moteur 2
#define enB 3
#define in3 4
#define in4 5

//ultrasonic
#define TRIG_PIN 6
#define ECHO_PIN 7


const int OBSTACLE_DISTANCE = 25; // Distance for obstacle detection in cm
Servo myServo;

enum go { LEFT, RIGHT };       // Enum to represent direction choices (LEFT or RIGHT)

/**
 * setup - Initialize the components
 * 
 * Description:
 *      This function sets up the pins as input or output, attaches the servo to its pin,
 */
void setup() {
    // Ultrasonic sensor pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Servo motor setup
    myServo.attach(10);         // Attach the servo motor to pin 14
    myServo.write(90);          // Initialize the servo at the center position (90°)

    // Motor control pins setup
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    Serial.begin(9600);         // Start serial communication at 9600 baud
}

/**
 * loop - Main program loop
 * 
 * Description:
 *      Continuously measures the distance using the ultrasonic sensor. If an obstacle
 *      is detected within the threshold distance, the car will rotate its servo to
 *      decide the best direction and adjust the motor direction accordingly.
 */
void loop() {
    float distance = measureDistance();

    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance <= OBSTACLE_DISTANCE) {
        Serial.println("Obstacle detected! Rotating servo to decide direction...");
        
        rotateServo180();
    }
    else
    {
        Serial.println("Path is clear! Moving forward...");
        setMotorsForward(); // Move forward if no obstacle is detected
    }

    delay(100); // Short delay for stability
}



/**
 * rotateServo180 - Rotate the servo to decide the best direction
 * 
 * Description:
 *      This function rotates the servo motor to look both left and right,
 *      measures the distances in those directions, determines the best direction,
 *      and sets the motor direction accordingly.
 */
void rotateServo180() {

    stopMotors();
    delay(500);  
    float rightDistance = lookRight();  // Measure distance to the right
    delay(500);
    float leftDistance = lookLeft();   // Measure distance to the left
    delay(500);

    go BestDirection = rightDirection(rightDistance, leftDistance); // Decide best direction
    setMotorDirection(BestDirection);  // Adjust motor direction
}



/**
 * lookRight - Rotate servo to the right and measure distance
 * 
 * Return: Distance to the right in cm
 * 
 * Description:
 *      This function rotates the servo to the right, measures the distance,
 *      and resets the servo to its center position.
 */
float lookRight() {
    myServo.write(0);  // Rotate servo to the right
    delay(500);
    float distance = measureDistance(); // Measure distance
    delay(100);
    myServo.write(90);  // Reset servo to center
    delay(100);
    return distance;
}

/**
 * lookLeft - Rotate servo to the left and measure distance
 * 
 * Return: Distance to the left in cm
 * 
 * Description:
 *      This function rotates the servo to the left, measures the distance,
 *      and resets the servo to its center position.
 */
float lookLeft() {
    myServo.write(180); // Rotate servo to the left
    delay(500);
    float distance = measureDistance(); // Measure distance
    delay(100);
    myServo.write(90);  // Reset servo to center
    delay(100);
    return distance;
}


/**
 * rightDirection - Determine the best direction based on distances
 * @RD: Distance to the right
 * @LD: Distance to the left
 * 
 * Return: RIGHT if the right distance is greater, otherwise LEFT
 * 
 * Description:
 *      Compares the distances to the left and right and decides the best
 *      direction to move to avoid the obstacle.
 */
go rightDirection(float RD, float LD) {
    if (RD >= LD) {
        Serial.print("Right distance is greater: ");
        Serial.println(RD);
        return RIGHT;
    }
    Serial.print("Left distance is greater: ");
    Serial.println(LD);
    return LEFT;
}



/**
 * measureDistance - Measure distance using the ultrasonic sensor
 * 
 * Return: The distance to the nearest object in cm
 * 
 * Description:
 *      This function triggers the ultrasonic sensor to send a pulse, calculates
 *      the time taken for the echo to return, and converts it into a distance.
 */
float measureDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    float duration = pulseIn(ECHO_PIN, HIGH);  // Measure the pulse duration
    return duration * 0.034 / 2;              // Convert duration to distance in cm
}


/**
 * setMotorDirection - Adjust the motor direction based on the decision
 * @D: The direction to move (LEFT or RIGHT)
 * 
 * Description:
 *      This function sets the motor pins to move the robot in the chosen direction
 *      for a predefined amount of time.
 */
void setMotorDirection(go D) {
    setMotorsBackward();
    delay(300);
    stopMotors();
    if (D == RIGHT) {
        rotationRight();
        Serial.println("Turning RIGHT...");
        
    } 
    else if (D == LEFT){
        rotationLeft();
        Serial.println("Turning LEFT...");   
    }
    else
    {
      Serial.println("Invalid direction! No action taken.");
    }

    delay(250);
}



/*motor functions*/

/**
 * stopMotors - Stops all motors by setting their control pins to LOW and disabling PWM speed.
 * 
 * Description:
 *      This function ensures that all motors stop immediately by setting their control pins
 *      to LOW and the enable pins (PWM) to 0. It effectively halts the robot's movement.
 */
void stopMotors() {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enA, 0);
    analogWrite(enB, 0);
}

/**
 * setMotorsBackward - Moves the robot backward at full speed.
 * 
 * Description:
 *      This function sets both motors to run in reverse by adjusting their control pins.
 *      It uses the `setMotorDirection` function to simplify setting the pins and speed.
 */
void setMotorsBackward() {
    setMotorDirection(in1, in2, enA, LOW, HIGH, 255); // Full speed backward
    setMotorDirection(in3, in4, enB, LOW, HIGH, 255); // Full speed backward
}

/**
 * setMotorsForward - Moves the robot forward at full speed.
 * 
 * Description:
 *      This function sets both motors to run forward by adjusting their control pins.
 *      It uses the `setMotorDirection` function to simplify setting the pins and speed.
 */
void setMotorsForward() {
    setMotorDirection(in1, in2, enA, HIGH, LOW, 255); // Full speed forward
    setMotorDirection(in3, in4, enB, HIGH, LOW, 255); // Full speed forward
}

/**
 * rotationRight - Rotates the robot to the right in place.
 * 
 * Description:
 *      This function sets one motor to move forward and the other to move backward.
 *      This creates a rotation effect that turns the robot to the right.
 */
void rotationRight() {
    // Set direction pins for both motors first
    digitalWrite(in1, LOW);   // Motor A backward
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);  // Motor B forward
    digitalWrite(in4, LOW);
    // Enable both motors at full speed
    analogWrite(enA, 255);
    analogWrite(enB, 255);
}

/**
 * rotationLeft - Rotates the robot to the left in place.
 * 
 * Description:
 *      This function sets one motor to move backward and the other to move forward.
 *      This creates a rotation effect that turns the robot to the left.
 */
void rotationLeft() {

        // Set direction pins for both motors first
    digitalWrite(in1, HIGH);   // Motor A backward
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);  // Motor B forward
    digitalWrite(in4, HIGH);
    // Enable both motors at full speed
    analogWrite(enA, 255);
    analogWrite(enB, 255);
}

/**
 * setMotorDirection - Configures the state and speed of a motor.
 * @pin1: First control pin for the motor
 * @pin2: Second control pin for the motor
 * @enPin: PWM pin to control motor speed
 * @state1: HIGH or LOW state for pin1
 * @state2: HIGH or LOW state for pin2
 * @speed: PWM speed value (0-255)
 * 
 * Description:
 *      This function sets the direction and speed of a motor by controlling the states
 *      of its control pins (pin1 and pin2) and applying PWM to the enable pin (enPin).
 */
void setMotorDirection(int pin1, int pin2, int enPin, int state1, int state2, int speed) {
    digitalWrite(pin1, state1);
    digitalWrite(pin2, state2);
    analogWrite(enPin, speed);
}
