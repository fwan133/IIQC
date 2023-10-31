//
// Created by feng on 2/07/23.
//

#ifndef DATA_QUALITY_CHECK_WAYPOINT_H
#define DATA_QUALITY_CHECK_WAYPOINT_H

#include "viewpoint.h"

class WayPoint {
public:
    WayPoint(uint Id, std::string SemanticLabel, CoordinateFrame coordinate_frame, float x, float y, float z, AngleUnit angle_unit, float yaw, float pitch, float roll){
        id = Id;
        semantic_label=SemanticLabel;
        viewpoint.UpdateAll(coordinate_frame, x, y, z, angle_unit, yaw, pitch, roll);
    };

    ~WayPoint(){};

    inline const uint& GetId() const {return id;};

    inline const std::string& GetSemanticLabel() const {return semantic_label;};

    inline const ViewPoint GetViewPoint() const {return viewpoint;};

    void updateViewPoint(const ViewPoint &viewpoint_){
        viewpoint.UpdateAll(viewpoint_);
    }

protected:
    uint id;
    std::string semantic_label;
    ViewPoint viewpoint;
};


#endif //DATA_QUALITY_CHECK_WAYPOINT_H
