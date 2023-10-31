//
// Created by feng on 2/07/23.
//

#ifndef DATA_QUALITY_CHECK_WAYPOINTS_MISSION_H
#define DATA_QUALITY_CHECK_WAYPOINTS_MISSION_H

#include "waypoint.h"
#include "viewpoint.h"
#include "captured_frame_collection.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

class WaypointsMission {
public:
    WaypointsMission();

    WaypointsMission(std::string file_name, std::string semantic_label, CoordinateFrame coordinate_frame,
                     AngleUnit angle_unit) {
        // Load waypoints file
        std::ifstream file(file_name);
        if (!file.is_open()) throw std::runtime_error("Failed to load waypoints file");

        // Assign coordinate transformation data

        // Assign waypoints
        std::string line;
        uint initial_index = 1;
        while (std::getline(file, line)) {
            if (line.back() == '\r') {
                line = line.substr(0, line.length() - 1);
            }
            std::istringstream iss(line);
            std::string value;
            std::vector<float> numbers;
            while (std::getline(iss, value, ',')) {
                float current_number = std::stof(value);
                numbers.push_back(current_number);
            }
            waypoint_list.push_back(
                    {initial_index, semantic_label, coordinate_frame, numbers[0], numbers[1], numbers[2], angle_unit,
                     numbers[3], numbers[4],
                     numbers[5]});
            initial_index++;
        }

        // Close the file
        file.close();
    };

    ~WaypointsMission() {};

    void PrintWaypoints() const {
        std::cout << "\n*********************** Display Waypoints ***************************" << std::endl;
        std::cout << "The flight mission includes " << waypoint_list.size() << " waypoints. They are: " << std::endl;

        for (const auto &waypoint: waypoint_list) {
            std::cout << "Waypoint " << waypoint.GetId() << ": Position in "
                      << waypoint.GetViewPoint().GetStrCoordinateFrame() << ": (" << waypoint.GetViewPoint().x() << ", "
                      << waypoint.GetViewPoint().y()
                      << ", " << waypoint.GetViewPoint().z() << "). Orientation in "
                      << waypoint.GetViewPoint().GetStrAngleUnit() << ": (" << waypoint.GetViewPoint().yaw() << ", "
                      << waypoint.GetViewPoint().pitch()
                      << ", " << waypoint.GetViewPoint().roll() << ")." << std::endl;
        }
        std::cout << "********************* Display Waypoints End *************************" << std::endl;
    };

    const int GetSize() const { return waypoint_list.size(); };

    const WayPoint *GetWaypointByIndex(uint index) const { return &waypoint_list.at(index - 1); }

    void UpdateViewPointAtIndex(uint index, ViewPoint viewpoint){
        waypoint_list[index-1].updateViewPoint(viewpoint);
    }


protected:
    std::vector<WayPoint> waypoint_list;
};

#endif //DATA_QUALITY_CHECK_WAYPOINTS_MISSION_H
