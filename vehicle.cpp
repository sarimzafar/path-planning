#include "vehicle.h"
using namespace std;

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double speed)
: x(x)
, y(y)
, s(s)
, d(d)
, speed(speed)
, yaw(yaw)
, created(true)
{}

void Vehicle::updateVehicleState(double x, double y, double s, double d, double yaw, double speed) {
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
    this->speed = speed;
}

vector<double> Vehicle::getVehicleXY() {
    vector<double> XY{this->x, this->y};
    return XY;
}

vector<double> Vehicle::getVehicleSD() {
    vector<double> SD{this->s, this->d};
    return SD;
}

vector<double> Vehicle::getVehicleSpeedYaw() {
    vector<double> SpeedYaw{this->speed, this->yaw};
    return SpeedYaw;
}

int Vehicle::getCurrentLane() {
    return int((this->d - 2)/4);
}
