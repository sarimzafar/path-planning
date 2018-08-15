#pragma once

#include <vector>

#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
public:
    Vehicle() = default;
    Vehicle(double x, double y, double s, double d, double yaw, double speed);
    virtual ~Vehicle() = default;

    void updateVehicleState(double x, double y, double s, double d, double yaw, double speed);
    std::vector<double> getVehicleXY();
    std::vector<double> getVehicleSD();
    std::vector<double> getVehicleSpeedYaw();
    int getCurrentLane();
    
private:
    bool created;
    double x;
    double y;
    double s;
    double d;
    double speed;
    double yaw;
};

#endif