#include <vector>
#include <math.h>
#include <cmath>
#include "vehicle.h"
#include <iostream>
#include <algorithm>
#include <cstdlib>

using namespace std;

// EVAL_DIFF - Difference between the current car and the car ahead in the SAME lane
const int EVAL_DIFF = 40;

// CLEARANCE - Min clearence required to ensure proper lane change
const int CLEARANCE = 20;

int getLaneNumber(double d) {
    int lane = -1; // Means its on the other lane

    if (d >= 0 && d <=4) {
        lane = 0;
    } else if (d > 4 && d <=8) {
        lane = 1;
    } else if (d > 8 && d <=12) {
        lane = 2;
    }

    return lane;
}

bool shouldChangeLane(vector<vector<double>> sensor_fusion, Vehicle vehicle, int prev_size) {
    bool changeLane = false;
    double current_s = (vehicle.getVehicleSD())[0];
    double current_d = (vehicle.getVehicleSD())[1];

    for (int i = 0; i < sensor_fusion.size(); ++i) {
        float neighbour_d = sensor_fusion[i][6];
        // If there is a car in the same lane
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double check_speed = sqrt(vx*vx + vy*vy);
        double check_car_s = sensor_fusion[i][5];

         // where the car would be after update
        check_car_s += ((double)prev_size*.02*check_speed);

        if (getLaneNumber(current_d) == getLaneNumber(neighbour_d)) {
            if((check_car_s > current_s) && ((check_car_s - current_s) < EVAL_DIFF)) {
                changeLane = true;
            }
        }
    }

    return changeLane;
}

vector<vector<double>> seperateCarsIntoLanes(vector<vector<double>> sensor_fusion, int currentLaneNumber) {
    vector<vector<double>> result (3);
    int neighbourLane = 0;

    for (int i = 0; i < sensor_fusion.size(); ++i) {
        float neighbour_d = sensor_fusion[i][6];
        // If there is a car in the same lane
        auto neighborLane = getLaneNumber(neighbour_d);
        // cout << "Printing neighborLane = " << neighborLane << endl;

        if (neighborLane >= 0 && neighborLane != currentLaneNumber) {
            result[neighborLane].push_back(sensor_fusion[i][5]);
        }
    }
    
    return result;
}

int evaluateLaneChange(vector< vector<double> > carInLanes, int currentLaneNumber, double current_s) {
    vector<double> lanes;
    
    for (int i = 0; i < carInLanes.size(); i++) {
        vector<double> neighbors_s = carInLanes[i];
        
        double diff = 0;

        if(neighbors_s.size() == 0) {
            diff = 0;
        } else if (neighbors_s.size() == 1) {
            diff = abs(current_s - neighbors_s[0]);
        } else {
            diff = abs(current_s - neighbors_s[0]);

            for (int j = 1; j < neighbors_s.size(); ++j) {
                diff = min(diff, abs(current_s - neighbors_s[j]));
            }
        }

        lanes.push_back(diff);
    }

    for (int i = 0; i < lanes.size(); i++) {
        cout << "Lane Number - " << i << " - Difference - " << lanes[i] << endl;
    }

    // Find the lane with max gap
    int laneChangeTo = distance(lanes.begin(), max_element(lanes.begin(), lanes.end()));

    // Cover 2 lane change cases - avoid 2 lane changes
    if (currentLaneNumber == 0 && laneChangeTo == 2) {
        laneChangeTo = 1;
    } else if (currentLaneNumber == 2 && laneChangeTo == 0) {
        laneChangeTo = 1;
    }

    if (laneChangeTo != currentLaneNumber) {
        cout << "Change Lane = " << laneChangeTo << endl;
    } else {
        cout << "Maintain Current Lane" << endl;
    }
    
    // Will only change lane if the gap is more than CLEARANCE - else slow down
    if (lanes[laneChangeTo] < CLEARANCE) {
        return -1;
    } 
    
    return laneChangeTo;
}

int getLane(vector<vector<double>> sensor_fusion, Vehicle vehicle, vector<double> previous_path_x, vector<double> previous_path_y) {
    bool laneChangeRequired = shouldChangeLane(sensor_fusion, vehicle, previous_path_x.size());
    int currentLaneNumber = getLaneNumber((vehicle.getVehicleSD())[1]);
    double current_s = (vehicle.getVehicleSD())[0];
    
    //If lane change is not required - remain in the same lane
    if (!laneChangeRequired) {
       return currentLaneNumber;
    }

    // Segregate surrounding cars into their specific lanes and gaps
    vector< vector<double> > carInLanes = seperateCarsIntoLanes(sensor_fusion, currentLaneNumber);

    // Evaluate cars in lane to find the most suitable lane
    return evaluateLaneChange(carInLanes, currentLaneNumber, current_s);
}