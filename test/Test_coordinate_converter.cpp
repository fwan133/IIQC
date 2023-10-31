#include "flight_control/waypoints_mission.h"
#include "CoordinateConverter.h"

int main(int argc, char **argv){
    // Parameters
    std::string filename="/home/feng/Code/catkin_ros/src/IIQC/data/flight_mission/ComplexGirderBridge/pier.csv";

    // WaypointsMission
    WaypointsMission waypoints_mission(filename, "Pier", CoordinateFrame::BRIDGE, DEGREE);
    CoordinateConverter coordinate_converter(-36.859501, 174.777554,0);
    coordinate_converter.SetBridge2ENU(0,0,0,0,0,0,AngleUnit::DEGREE);
    coordinate_converter.SetLocal2ENU(-10, -7.5, 0, 0, 0, 0, AngleUnit::RADIAN);
    // Check the Coordinate Convert
    ViewPoint viewpoint_org = waypoints_mission.GetWaypointByIndex(1)->GetViewPoint();
    viewpoint_org.printInfo();

    ViewPoint viewpoint_new;
    coordinate_converter.ConvertBRIDGE2LOCAL(viewpoint_org,viewpoint_new);
    viewpoint_new.RAD2DEGREE();
    viewpoint_new.printInfo();

    ViewPoint viewpoint_new2;
    coordinate_converter.ConvertBRIDGE2GPS(viewpoint_org,viewpoint_new2);
    viewpoint_new2.RAD2DEGREE();
    viewpoint_new2.printInfo();


//    waypoints_mission.PrintWaypoints();
}
