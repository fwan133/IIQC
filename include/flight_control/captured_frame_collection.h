//
// Created by feng on 2/07/23.
//

#ifndef DATA_QUALITY_CHECK_CAPTUREDFRAMECOLLECTION_H
#define DATA_QUALITY_CHECK_CAPTUREDFRAMECOLLECTION_H

#include <iostream>
#include <fstream>
#include <filesystem>
#include <sys/stat.h>
#include <string>

#include "captured_frame.h"

namespace fs = std::filesystem;

class CapturedFrameCollection {
public:
    CapturedFrameCollection() {};

    CapturedFrameCollection(const CapturedFrameCollection &other) {
        this->captured_frame_collection = other.captured_frame_collection;
    };

    CapturedFrameCollection(std::string file_folder_path, std::string file_name, std::string semantic_label, int length,
                            bool load_img) {
        // Read the txt file
        fs::path directory_path = file_folder_path;
        std::string txt_directory;
        try {
            txt_directory = file_folder_path + "/" + file_name;
        } catch (const fs::filesystem_error &ex) {
            std::cerr << "Error accessing the configuration file." << ex.what() << std::endl;
        }

        // Read the txt file
        std::ifstream inputFile(txt_directory);
        std::string line;
        bool if_head = true;
        int index = 0;
        while (std::getline(inputFile, line)) {
            if (length != 0 && index >= length) {
                break;
            }

            if (if_head) {
                if_head = false;
                continue;
            }
            std::vector<std::string> tokens;
            std::istringstream lineStream(line);
            std::string token;

            while (std::getline(lineStream, token, ',')) {
                // Process each token here (e.g., store it in a vector)
                tokens.push_back(token);
            }

            // Generate Frame
            CoordinateFrame coor_frame;
            if (tokens[1] == "ENU") {
                coor_frame = CoordinateFrame::ENU;
            }
            AngleUnit angle_unit;
            if (tokens[5] == "RADIAN") {
                angle_unit = AngleUnit::RADIAN;
            } else if (tokens[5] == "DEGREE") {
                angle_unit = AngleUnit::DEGREE;
            } else {
                angle_unit = AngleUnit::RADIAN;
            }
            WayPoint waypoint(std::stoi(tokens[0]), semantic_label, coor_frame, std::stod(tokens[2]),
                              std::stod(tokens[3]), std::stod(tokens[4]), angle_unit, std::stod(tokens[6]),
                              std::stod(tokens[7]), std::stod(tokens[8]));
            ViewPoint viewpoint = waypoint.GetViewPoint();
            CapturedFrame frame(waypoint, 0.0);
            frame.SetGeoInfo(viewpoint);

            /// LoadImage
            frame.SetImgPath(file_folder_path + "/" + tokens[0] + "_colour.jpg");
            if (load_img) {
                frame.SetColourImg(cv::imread(file_folder_path + "/" + tokens[0] + "_colour.jpg"));
            }

            frame.SetGroudTruthPose(
                    Pose6d(std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]), std::stod(tokens[12]),
                           std::stod(tokens[13]), std::stod(tokens[14]), std::stod(tokens[15])));

            /// Set Estimated Pose
            frame.SetEstPose(
                    Pose6d(std::stod(tokens[16]), std::stod(tokens[17]), std::stod(tokens[18]), std::stod(tokens[19]),
                           std::stod(tokens[20]), std::stod(tokens[21]), std::stod(tokens[22])));

            captured_frame_collection.push_back(frame);

            index++;
        }
    };

    CapturedFrameCollection(std::string semantic_label, std::string file_folder_path, int length, bool load_img) {
        // Read the txt file
        fs::path directory_path = file_folder_path;
        std::string txt_directory;
        try {
            for (const auto &entry: fs::directory_iterator(directory_path)) {
                if (entry.is_regular_file() && entry.path().extension() == ".txt") {
                    txt_directory = entry.path();
                }
            }
        } catch (const fs::filesystem_error &ex) {
            std::cerr << "Error accessing the configuration file." << ex.what() << std::endl;
        }

        // Read the txt file
        std::ifstream inputFile(txt_directory);
        std::string line;
        bool if_head = true;
        int index = 0;
        while (std::getline(inputFile, line)) {
            if (length != 0 && index >= length) {
                break;
            }

            if (if_head) {
                if_head = false;
                continue;
            }
            std::vector<std::string> tokens;
            std::istringstream lineStream(line);
            std::string token;

            while (std::getline(lineStream, token, ',')) {
                // Process each token here (e.g., store it in a vector)
                tokens.push_back(token);
            }

            // Generate Frame
            CoordinateFrame coor_frame;
            if (tokens[1] == "ENU") {
                coor_frame = CoordinateFrame::ENU;
            }
            AngleUnit angle_unit;
            if (tokens[5] == "RADIAN") {
                angle_unit = AngleUnit::RADIAN;
            } else if (tokens[5] == "DEGREE") {
                angle_unit = AngleUnit::DEGREE;
            } else {
                angle_unit = AngleUnit::RADIAN;
            }
            WayPoint waypoint(std::stoi(tokens[0]), semantic_label, coor_frame, std::stod(tokens[2]),
                              std::stod(tokens[3]), std::stod(tokens[4]), angle_unit, std::stod(tokens[6]),
                              std::stod(tokens[7]), std::stod(tokens[8]));
            ViewPoint viewpoint = waypoint.GetViewPoint();
            CapturedFrame frame(waypoint, 0.0);
            frame.SetGeoInfo(viewpoint);

            /// LoadImage
            frame.SetImgPath(file_folder_path + "/" + tokens[0] + "_colour.jpg");
            if (load_img) {
                frame.SetColourImg(cv::imread(file_folder_path + "/" + tokens[0] + "_colour.jpg"));
            }

            frame.SetGroudTruthPose(
                    Pose6d(std::stod(tokens[9]), std::stod(tokens[10]), std::stod(tokens[11]), std::stod(tokens[12]),
                           std::stod(tokens[13]), std::stod(tokens[14]), std::stod(tokens[15])));
            captured_frame_collection.push_back(frame);

            index++;
        }
    };

    ~CapturedFrameCollection() {};

    const int GetSize() const { return captured_frame_collection.size(); };

    const ViewPoint getViewPointByIndex(uint index) const {
        return captured_frame_collection[index - 1].GetGeoCoordinate();
    }

    const CapturedFrame getFrameByIndex(uint index) const {
        return captured_frame_collection[index - 1];
    }

    void updateGeoCoordinateByIndex(uint index, ViewPoint &viewpoint) {
        captured_frame_collection[index - 1].SetGeoInfo(viewpoint);
    }

    void AddFrame(CapturedFrame &captured_frame) {
        captured_frame_collection.push_back(captured_frame);
    };

    void setEstByIndex(uint &index, Eigen::Transform<double, 3, 1> T_est) {
        captured_frame_collection[index - 1].SetEstPose(T_est);
    }

    void PrintInfo() {
        std::cout << "\n******************* Displayed Captured Data *********************" << std::endl;
        std::cout << "A total of " << captured_frame_collection.size() << " viewpoints being captured. They are: "
                  << std::endl;
        for (auto captured_frame: captured_frame_collection) {
            std::cout << "Waypoint " << captured_frame.GetId() << " for " << captured_frame.GetSemanticLabel() << ": "
                      << captured_frame.GetGeoCoordinate().GetStrCoordinateFrame() << std::fixed << std::setprecision(7)
                      << " {"
                      << captured_frame.GetGeoCoordinate().latitude() << ", "
                      << captured_frame.GetGeoCoordinate().longitude() << ", "
                      << captured_frame.GetGeoCoordinate().altitude() << "} "
                      << captured_frame.GetGeoCoordinate().GetStrAngleUnit() << " {"
                      << captured_frame.GetGeoCoordinate().yaw() << ", "
                      << captured_frame.GetGeoCoordinate().pitch() << ", "
                      << captured_frame.GetGeoCoordinate().roll() << "}"
                      << std::endl;
        }
        std::cout << "****************** Displayed Captured Data End ********************" << std::endl;
    }

    void SaveToFile(std::string parent_folder_path, bool img_save) {
        //
        std::string file_folder_path =
                parent_folder_path + this->captured_frame_collection.begin()->GetSemanticLabel() + "/";

        // Create the folder if not exist.
        if (!fs::exists(file_folder_path)) { fs::create_directory(file_folder_path); }

        std::ofstream output_file(
                file_folder_path + this->captured_frame_collection.begin()->GetSemanticLabel() + ".txt");
        if (!output_file) {
            std::cerr << "Failed to create the target trajectory" << std::endl;
            return;
        }

        // Form the strstream
        output_file
                << "#ID, Target, Global Frame, global_x, global_y, global_z, Angle Unit, yaw, pitch, row, Ground Truth Pose x, y, z, qx, qy, qz, qw, Estimated Pose x, y, z, qx, qy, qz, qw"
                << std::endl;
        for (auto captured_frame: captured_frame_collection) {
            output_file << captured_frame.GetId() << ", " << captured_frame.GetGeoCoordinate().GetStrCoordinateFrame()
                        << ", " << std::fixed << std::setprecision(7) << captured_frame.GetGeoCoordinate().latitude()
                        << ", " << captured_frame.GetGeoCoordinate().longitude() << ", "
                        << captured_frame.GetGeoCoordinate().altitude() << ", "
                        << captured_frame.GetGeoCoordinate().GetStrAngleUnit() << ", " << std::fixed
                        << std::setprecision(3) << captured_frame.GetGeoCoordinate().yaw() << ", "
                        << captured_frame.GetGeoCoordinate().pitch() << ", " << captured_frame.GetGeoCoordinate().roll()
                        << ", " << captured_frame.GetGroundTruthPose().position.x() << ", "
                        << captured_frame.GetGroundTruthPose().position.y() << ", "
                        << captured_frame.GetGroundTruthPose().position.z() << ", "
                        << captured_frame.GetGroundTruthPose().orientation.x() << ", "
                        << captured_frame.GetGroundTruthPose().orientation.y() << ", "
                        << captured_frame.GetGroundTruthPose().orientation.z() << ", "
                        << captured_frame.GetGroundTruthPose().orientation.w() << ", "
                        << captured_frame.GetEstimatedPose().position.x() << ", "
                        << captured_frame.GetEstimatedPose().position.y() << ", "
                        << captured_frame.GetEstimatedPose().position.z() << ", "
                        << captured_frame.GetEstimatedPose().orientation.x() << ", "
                        << captured_frame.GetEstimatedPose().orientation.y() << ", "
                        << captured_frame.GetEstimatedPose().orientation.z() << ", "
                        << captured_frame.GetEstimatedPose().orientation.w() << std::endl;

            if (img_save) {
                bool result1 = cv::imwrite(file_folder_path + std::to_string(captured_frame.GetId()) + "_colour.jpg",
                                           captured_frame.GetColourImage());
//            bool result2=cv::imwrite(file_folder_path + std::to_string(captured_frame.GetId()) + "_depth.jpg", captured_frame.GetDepthImage());
                if (!result1) {
                    std::cerr << "Failed to save image at Waypoint " << captured_frame.GetId() << std::endl;
                }
            }
        }

        output_file.close();
        std::cout << "\nSuccessfully Saved All Images Info." << std::endl;
    };

protected:
    std::vector<CapturedFrame> captured_frame_collection;
};


#endif //DATA_QUALITY_CHECK_CAPTUREDFRAMECOLLECTION_H
