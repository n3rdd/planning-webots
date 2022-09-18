// File:          controller2.cpp
// Date:
// Description:
// Author:
// Modifications:
#include <string>
#include <limits>
// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using std::cin;
using std::cout;
using std::endl;

const std::string motor_names[4] = {"motor1", "motor2", "motor3", "motor4"};
const double v = 10;
const double v2 = 10;
const double velocity_forward[4] = {v, v, v, v};
const double velocity_backward[4] = {-v, -v, -v, -v};
const double velocity_leftward[4] = {v, -v, v, -v};
const double velocity_rightward[4] = {-v, v, -v, v};
const double velocity_leftcircle[4] = {v2, -v2, -v2, v2};
const double velocity_rightcircle[4] = {-v2, v2, v2, -2};

void mapping(Robot* robot, Keyboard keyboard) {
  cout << "mapping starts..." << endl;

  keyboard.enable(1);
  Motor *motors[4] = {
    robot->getMotor(motor_names[0]),
    robot->getMotor(motor_names[1]),
    robot->getMotor(motor_names[2]),
    robot->getMotor(motor_names[3])
  };
  for (int i = 0; i < 4; ++i) {
    motors[i]->setPosition(std::numeric_limits<double>::infinity());
    motors[i]->setVelocity(0.0);
  }
  int timeStep = (int)robot->getBasicTimeStep();
  cout << "UP: " << keyboard.UP << endl;
  while (robot->step(timeStep) != -1) {
    auto key = keyboard.getKey();
    // cout << key << " pressed" << endl;
    switch (key) {
      case keyboard.UP: {
        for (int i = 0; i < 4; ++i) {
          cout << "UP pressed" << endl;
          motors[i]->setVelocity(velocity_forward[i]);
        }
        break;
      }
      case keyboard.DOWN: {
        for (int i = 0; i < 4; ++i) {
          cout << "DOWN pressed" << endl;
          motors[i]->setVelocity(velocity_backward[i]);
        }
        break;
      }
      case keyboard.LEFT: {
        for (int i = 0; i < 4; ++i) {
          cout << "LEFT pressed" << endl;
          motors[i]->setVelocity(velocity_leftward[i]);
        }
        break;
      }
      case keyboard.RIGHT: {
        for (int i = 0; i < 4; ++i) {
          cout << "RIGHT pressed" << endl;
          motors[i]->setVelocity(velocity_rightward[i]);
        }
        break;
      }

    }
    
  }

}

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Keyboard keyboard;
  // keyboard.enable(100);

  std::string cmd = "";
  mapping(robot, keyboard);
  while (true) {
    // cout << "$ ";
    // cin >> cmd;
    // if (cmd == "exit") {
    //   break;
    // }
    // if (cmd == "mapping") {
    //   mapping(robot, keyboard);
    // }
    break;
    
  }
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    break;
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
