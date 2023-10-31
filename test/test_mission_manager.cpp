//
// Created by feng on 3/07/23.
//

#include "flight_control/flight_mission_manager.h"
#include "flight_control/captured_frame_collection.h"
#include "CoordinateConverter.h"

int main(int argc, char **argv) {
    // Parameters
    std::string filename="/home/feng/Code/catkin_ros/src/IIQC/data/flight_mission/Building/building2.csv";

    // WaypointsMission
    WaypointsMission waypoints_mission(filename, "Building3", BRIDGE, DEGREE);
    CoordinateConverter coordinate_converter(-36.8593715, 174.7733865,0);
    coordinate_converter.SetBridge2ENU(0,0,0,0,0,0,AngleUnit::DEGREE);
    coordinate_converter.SetLocal2ENU(-15, -27, 0, 0, 0, 0, AngleUnit::RADIAN);
    for (int index=1; index <= waypoints_mission.GetSize(); index++){
        ViewPoint viewpoint_new;
        coordinate_converter.ConvertBRIDGE2LOCAL(waypoints_mission.GetWaypointByIndex(index)->GetViewPoint(),viewpoint_new);
        viewpoint_new.RAD2DEGREE();
        waypoints_mission.UpdateViewPointAtIndex(index,viewpoint_new);
    }
    waypoints_mission.PrintWaypoints();

    // Execute Waypoint Mission
    ros::init(argc, argv, "FlightMission");
    FlightMissionManager flight_mission("typhoon_h480_0", "/typhoon_h480_0/camera/image_raw");

    CapturedFrameCollection captured_frame_collection=flight_mission.ExecuteWaypointMission(waypoints_mission, "cgo3_camera_link");  //cgo3_camera_link, monocular_camera

    for (int index=1; index <= captured_frame_collection.GetSize(); index++){
        ViewPoint viewpoint_new;
        coordinate_converter.ConvertGPS2ENU(captured_frame_collection.getViewPointByIndex(index),viewpoint_new);
        captured_frame_collection.updateGeoCoordinateByIndex(index,viewpoint_new);
    }
    captured_frame_collection.PrintInfo();
    captured_frame_collection.SaveToFile("/home/feng/Code/catkin_ros/src/IIQC/data/CapturedData/Building/",true);

    return 0;
}