# ObstacleAvoidanceCar
*ObstacleAvoidanceCar* is an autonomous car project built with an atmega328p, designed to navigate and avoid obstacles. Using ultrasonic sensors for distance measurement, the car adjusts its path by controlling motors and servos to move forward, backward, or turn in the right direction. This project is a great way to learn about robotics and autonomous navigation.

## Authors
- Oussama Laamri (me)
- Saad Larhrib (my friend)

## Features
- Obstacle Detection: Ultrasonic sensors to measure distances and detect obstacles.
- Autonomous Navigation: Car adjusts its path to avoid collisions.
- Motor Control: Full control over motors for movement and turns.
- Servo Steering: Servo-controlled steering for precise turns.
- ATmega328P Microcontroller: Acts as the brain of the car, managing sensor data processing, motor control, and decision-making.
## Components
- atmega328p - Main controller for the car's operations.
- HC-SR04 Ultrasonic sensor - For detecting obstacles in the car's path.
- Two DC motors & L293D motor driver - To drive the car forward, backward, and rotate.
- SG90 Servo motor - To control the steering of the car.
- 9V battery - Powers the car's electronic components.

## How It Works
The car moves forward until it detects an obstacle using an ultrasonic sensor. Once it does, it looks left and right using servo motor, checks which direction has more space, and takes the best path.
