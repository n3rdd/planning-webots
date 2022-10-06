// File:          controller2.cpp
// Date:
// Description:
// Author:
// Modifications:
#include <string>
#include <limits>
#include <memory>
#include <iostream>
#include <cmath>
#include <utility>
// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Display.hpp>
#include <webots/GPS.hpp>
#include <webots/Lidar.hpp>

#include <mapping.hpp>
#include <planning.hpp>
#include <controller.hpp>


// All the webots classes are defined in the "webots" namespace
using namespace webots;
using std::abs;
using std::cin;
using std::cout;
using std::endl;
using std::make_pair;

const std::string motor_names[4] = {"motor1", "motor2", "motor3", "motor4"};
const double v = 7;
const double v2 = 10;
const double velocity_forward[4] = {v, v, v, v};
const double velocity_backward[4] = {-v, -v, -v, -v};
const double velocity_leftward[4] = {v, -v, v, -v};
const double velocity_rightward[4] = {-v, v, -v, v};
const double velocity_leftcircle[4] = {v2, -v2, -v2, v2};
const double velocity_rightcircle[4] = {-v2, v2, v2, -2};

double velocity_x[4] = {0, 0, 0, 0};
double velocity_y[4] = {0, 0, 0, 0};
double velocity_circle[4] = {0, 0, 0, 0};


void setVelocity(int key, Keyboard keyboard, Motor* motors[]) {
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
      case ' ': {
        for (int i = 0; i < 4; ++i) {
          cout << "space pressed" << endl;
          motors[i]->setVelocity(0.);
        }
        break;
      }
    }
}

// void mappingAdd(const double max_range, const int horizontal_resolution, const float* range_image, const double* pos) {
//   double x = pos[0], y = pos[1];
//   for (int i = 0; i < horizontal_resolution, ++i) {
//     if (sign(range_image[i] - max_range) == 0) continue;
//     auto [ny, nx] = worldToMap(x + range_image[i] * cos(-i * unit),
//                                y + range_image[i] * sin(-i * unit));
//     if (nx >= 0 && )
//   }
// }

void updateMapDisplay(Display* display, vector<vector<int>> occupancy_grid_map, int width, int height) {
  display->setColor(0xBB2222);
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      if (occupancy_grid_map[i][j] == 1) {
        display->drawPixel(j, i);
      }
    }
  }
}

void updateRobotDisplay(Display* display, int gx, int gy) {
  display->setColor(0x222222); 
  display->drawPixel(gx, gy);

}

// void updatePathDisplay(Display* display, vector<Node> path) {
void updatePathDisplay(Display* display, vector<pair<int, int> > path) {
  display->setColor(0x22BBBB);
  for (auto& pos: path) {
    cout << pos.first << " " << pos.second << endl;
    display->drawPixel(pos.first, pos.second);
  }
}

char getKeyX(double x_control, int cur_x) {
  double diff = x_control - cur_x;
  if (abs(diff) <= 1)  return ' ';
  return diff > 0? 'S': 'W';
}

char getKeyY(double y_control, int cur_y) {
  double diff = y_control - cur_y;
  if (abs(diff) <= 1)  return ' ';
  return diff > 0? 'A': 'D';
}

void mapping(Robot* robot) {
  // void mapping(std::unique_ptr<Robot>& robot) {
  cout << "mapping starts..." << endl;
  // keyboard
  Keyboard keyboard;
  keyboard.enable(1);

  // GPS
  GPS* gps = robot->getGPS("gps");
  gps->enable(1);

  InertialUnit *inertial = robot->getInertialUnit("yaw");
  inertial->enable(1);

  // LiDAR
  Lidar *lidar = robot->getLidar("lidar");
  lidar->enable(1);
  lidar->enablePointCloud();
  const double lidar_max_range = lidar->getMaxRange();
  const int horizontal_resolution = lidar->getHorizontalResolution();
  const float* range_image = lidar->getRangeImage();

  // display
  Display *display = robot->getDisplay("display");
  display->setOpacity(1);
  const int display_width = display->getWidth();
  const int display_height = display->getHeight();

  // motor
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

  const double floor_width = 5;
  const double floor_height = 5;
  const int mapping_update_period = 16;
  int counter = 0;  // for mapping update

  Mapping mapping(
      horizontal_resolution, 
      lidar_max_range,
      display_height,
      display_width,
      floor_height,
      floor_width);

  
  
  const double front = 1.57;
  bool move_pid = false;
  vector<pair<int, int> > path;
  vector<pair<int, int> > pos_for_stucked;
  int idx = -1; // path idx
  PidController pid;
  pid.init();

  int timeStep = (int)robot->getBasicTimeStep();
  while (robot->step(timeStep) != -1) {
    const double *pos = gps->getValues();
    double x = pos[0], y = pos[1];
    const double *yaw = inertial->getRollPitchYaw();
  
    // cout << "pos: " << x << ", " << y << endl;
    //////////////////////////
    // mapping
    //////////////////////////
    range_image = lidar->getRangeImage();
    mapping.updateOccupancyCount(range_image, x, y);
    ++counter;
    if (counter == mapping_update_period) {
      counter = 0;
      mapping.updateMap();
      updateMapDisplay(display, mapping.occupancy_grid_map_, mapping.width_, mapping.height_);
      auto [rx, ry] = mapping.worldToMap(x, y);
      updateRobotDisplay(display, rx, ry);
    }

    int key = keyboard.getKey();
    // cout << key << " pressed" << endl;

    //////////////////////////
    // planning
    //////////////////////////
    /* 找不到路径原因可能是起始点在 OGM 中为 1 */
    auto [gx, gy] = mapping.worldToMap(-2, -2);
    if (key == 'P') {
      // auto [sx, sy] = mapping.worldToMap(1.2, 2);
      auto [sx, sy] = mapping.worldToMap(pos[0], pos[1]);
      // auto [gx, gy] = mapping.worldToMap(-2, -2);
      cout << sx << " " << sy << endl;
      cout << gx << " " << gy << endl;
      // Node start {sx, sy};
      // Node goal{gx, gy};  // Node goal = Node(gx, gy);
      AstarPlanner astar_planner(mapping.occupancy_grid_map_, display_width, display_height);
      // timeStep = (int)robot->getBasicTimeStep();
      // vector<Node> path;
      
      
      display->setColor(0x999999);
      display->drawPixel(sx, sy);
      display->drawPixel(gx, gy);
      path = astar_planner.plan(sx, sy, gx, gy);
      updatePathDisplay(display, path);
      
      idx = path.size() - 2;
    }

    //////////////////////////
    // control
    //////////////////////////
    if (key == 'O') {
      move_pid = true;
    } else if (key == 'L') {
      move_pid = false;
    }

    if (move_pid) {
      cout << "move pid" << endl;

      
      
      cout << "path size: " << path.size() << endl;
      Point target_point(path[idx].first, path[idx].second);
      pid.setPoint(target_point);

      auto [cur_x, cur_y] = mapping.worldToMap(pos[0], pos[1]);
      double x_control = pid.calcX(cur_x) + cur_x;
      double y_control = pid.calcY(cur_y) + cur_y;

      pos_for_stucked.push_back(make_pair(cur_x, cur_y));

      if (fabs(yaw[0] - front) >= 0.05) {
        int dir = (yaw[0] < front && yaw[0] > -front ? -1 : 1);
        for (int i = 0; i < 4; ++i) {
            velocity_circle[i] = velocity_rightcircle[i] * dir;
            motors[i]->setVelocity(velocity_circle[i]);
        }
        continue;
      } else {
        for (int i = 0; i < 4; ++i) {
            motors[i]->setVelocity(0.);
        }
      }

      int same_pos_count = 0;
      for (int i = pos_for_stucked.size()-2; i >= 1; --i) {
        if (pos_for_stucked[i].first == cur_x && pos_for_stucked[i].second == cur_y) {
          same_pos_count += 1;
        }
      }
      if (same_pos_count >= 15) {
        for (int i = 0; i < 4; ++i) {
            motors[i]->setVelocity(velocity_backward[i]);
        }
      }

      // 根据反馈量用键盘控制
      char key_x_control = getKeyX(x_control, cur_x);
      char key_y_control = getKeyY(y_control, cur_y);

      cout << "cur: " << cur_x << ", " << cur_y << endl;
      cout << "target_point: " << target_point.x_ << ", " << target_point.y_ << endl;
      cout << "key_x_control: " << key_x_control << endl;
      cout << "key_y_control: " << key_y_control << endl;

      // 根据键位设置电机速度
      switch(key_x_control) {
          case 'W': {
              for (int i = 0; i < 4; i++) {
                  velocity_x[i] = velocity_forward[i];
              }
              break;
          }
          case 'S': {
              for (int i = 0; i < 4; i++) {
                  velocity_x[i] = velocity_backward[i];
              }
              break;
          }
          case ' ' : {
              for (int i = 0; i < 4; i++) {
                  velocity_x[i] = 0;
              }
          }
      }
      switch(key_y_control) {
          case 'A': {
              for (int i = 0; i < 4; i++) {
                  velocity_y[i] = velocity_leftward[i];
              }
              break;
          }
          case 'D': {
              for (int i = 0; i < 4; i++) {
                  velocity_y[i] = velocity_rightward[i];
              }
              break;
          }
          case ' ' : {
              for (int i = 0; i < 4; i++) {
                  velocity_y[i] = 0;
              }
          }
      }
      for (int i = 0; i < 4; i++) {
          motors[i]->setVelocity(velocity_x[i] + velocity_y[i]);
      }

      if ((abs(target_point.y_ - cur_y) <= 3) && (abs(target_point.x_ - cur_x) <= 3) && idx >= 0) {
          idx -= 1;
      }

      if (((abs(gy - cur_y) <= 1) && (abs(gx - cur_x) <= 1)) || idx == 0) {
          cout << "****** Reach the goal *****" << endl;
          for (int i = 0; i < 4; i++) {
            motors[i]->setVelocity(0);
          }
          move_pid = false;
      }
      
    }
    
    //// else move normally 
    setVelocity(key, keyboard, motors);

  }



}

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  // auto robot = std::make_unique<Robot>();

  // get the time step of the current world.
  // int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  

  mapping(robot);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  // while (robot->step(timeStep) != -1) {
  //   break;
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  // };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
