#include <vector>
#include "vehicle.h"

int getLaneNumber(double d);
bool shouldChangeLane(std::vector<std::vector<double>> sensor_fusion, Vehicle vehicle, int prev_size);
int getLane(std::vector<std::vector<double>> sensor_fusion, Vehicle vehicle, std::vector<double> previous_path_x, std::vector<double> previous_path_y);
int evaluateLaneChange(std::vector<std::vector<double>> carInLanes, int currentLaneNumber, double current_s);
std::vector<std::vector<double>> seperateCarsIntoLanes(std::vector<std::vector<double>> sensor_fusion, int currentLaneNumber);
