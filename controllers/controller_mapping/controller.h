#include <utility>

using std::pair;

class Point {
public:
    double x_, y_;
    Point(double x, double y): x_(x), y_(y) {}
    Point(): x_(0.), y_(0.) {}
};

class PidController {
public:
    Point target_point_;
    Point sum_error_;
    
    double kp_, ki_, kd_;
    Point prev_error_;  // error[-1]
    Point prev2_error_; // error[-2]

    PidController() {
        kp_ = ki_ = kd_ = 0;
        target_point_.x_ = target_point_.y_ = 0;
        sum_error_.x_ = sum_error_.y_ = 0;
        prev_error_.x_ = prev_error_.y_ = 0;
        prev2_error_.x_ = prev2_error_.y_ = 0;
    }

    void setPoint(Point target_point);
    void init();
    double calcX(int cur_x);
    double calcY(int cur_y);

};

void PidController::setPoint(Point target_point) {
    target_point_.x_ = target_point.x_;
    target_point_.y_ = target_point.y_;
}

void PidController::init() {
    kp_ = 0.97;
    ki_ = 0.05;
    kd_ = 50;
    target_point_.x_ = target_point_.y_ = 0;
    sum_error_.x_ = sum_error_.y_ = 0;
    prev_error_.x_ = prev_error_.y_ = 0;
    prev2_error_.x_ = prev2_error_.y_ = 0;
}

double PidController::calcX(int cur_x) {
    double error_x, diff_error_x;

    error_x = target_point_.x_ - cur_x;
    sum_error_.x_ += error_x;
    if (sum_error_.x_ > 10)  sum_error_.x_ = 10;
    if (sum_error_.x_ < -10)  sum_error_.x_ = -10;
    
    diff_error_x = error_x - prev_error_.x_;
    prev_error_.x_ = error_x;
    
    return kp_ * error_x + ki_ * sum_error_.x_ + kd_ * diff_error_x;
}

double PidController::calcY(int cur_y) {
    double error_y, diff_error_y;

    error_y = target_point_.y_ - cur_y;
    sum_error_.y_ += error_y;
    if (sum_error_.y_ > 10)  sum_error_.y_ = 10;
    if (sum_error_.y_ < -10)  sum_error_.y_ = -10;
    
    diff_error_y = error_y - prev_error_.y_;
    prev_error_.y_ = error_y;
    
    return kp_ * error_y + ki_ * sum_error_.y_ + kd_ * diff_error_y;
}