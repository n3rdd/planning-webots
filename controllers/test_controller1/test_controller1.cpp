// File:          test_controller1.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Display.hpp>
#include <webots/GPS.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include <limits>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv)
{
    // create the Robot instance.
    Robot *robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor *motor = robot->getMotor("motorname");
    //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
    //  ds->enable(timeStep);
    Lidar *lidar = robot->getLidar("lidar");
    Motor *motors[4];

    char wheelsNames[4][8] = {"motor1", "motor2", "motor3", "motor4"};
    double speed1[4];
    double speed2[4];
    double speed3[4];

    // 初始化电机和距离传感器
    for (int i = 0; i < 4; i++)
    {
        motors[i] = robot->getMotor(wheelsNames[i]);
        motors[i]->setPosition(std::numeric_limits<double>::infinity());
        motors[i]->setVelocity(0.0);
        speed1[i] = 0;
        speed2[i] = 0;
        speed3[i] = 0;
    }
    const double velocity = 15;
    const double velocity2 = 15;

    double speed_forward[4] = {velocity, velocity, velocity, velocity};
    double speed_backward[4] = {-velocity, -velocity, -velocity, -velocity};
    double speed_leftward[4] = {velocity, -velocity, velocity, -velocity};
    double speed_rightward[4] = {-velocity, velocity, -velocity, velocity};

    double speed_leftCircle[4] = {velocity2, -velocity2, -velocity2, velocity2};
    double speed_rightCircle[4] = {-velocity2, velocity2, velocity2,
                                   -velocity2};
    lidar->enable(1);
    lidar->enablePointCloud();
    const int lidar_point_count = lidar->getHorizontalResolution();
    double lidar_max_range = lidar->getMaxRange();
    const float *dis = lidar->getRangeImage();

    int count = 0;
    for (int i = 0; i < 4; i++) {
        motors[i]->setVelocity(speed_forward[i]);
    }
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(timeStep) != -1) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();

        // Process sensor data here.

        // Enter here functions to send actuator commands, like:
        //  motor->setPosition(10.0);
        dis = lidar->getRangeImage();
        for (int i = 0; i < lidar_point_count; i++) {
            cout << dis[i] - lidar_max_range << endl;
            if (dis[i] - lidar_max_range != 0) {
                for (int i = 0; i < 4; i++) {
                    motors[i]->setVelocity(speed_leftward[i]);
                }
            }
        }

        if (count == 1050) {
            break;
        }

        count += 1;
    }

     

// Enter here exit cleanup code.

    delete robot;
    return 0;
}
