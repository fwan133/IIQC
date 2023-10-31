#ifndef IIQC_SEMANTICPC_H
#define IIQC_SEMANTICPC_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class SemanticPC {
    class Point {
    public:
        float x, y, z;
        int r, g, b;
        std::string semantic_label;

        Point(float x, float y, float z, int r, int g, int b, const std::string& label)
                : x(x), y(y), z(z), r(r), g(g), b(b), semantic_label(label) {}

        void print() {
            std::cout << "x: " << x << ", y: " << y << ", z: " << z
                      << ", r: " << r << ", g: " << g << ", b: " << b
                      << ", label: " << semantic_label << std::endl;
        }
    };

public:
    SemanticPC(){};
    SemanticPC(std::string filepath){
        std::ifstream file(filepath);
        if (!file.is_open()) {
            std::cerr << "Failed to open the file." << std::endl;
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            // Skip comment lines (like headers)
            if (line.find('#') == 0) {
                continue;
            }

            std::istringstream iss(line);
            float x, y, z;
            int r, g, b;
            std::string label;

            if (iss >> x >> y >> z >> r >> g >> b >> label) {
                points.emplace_back(x, y, z, r, g, b, label);
            }
        }

        // Close the file after reading
        file.close();
    }


    void Visualise() const {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        // ... (code to populate cloud)
        for (Point p : points) {
            pcl::PointXYZRGB pcl_point(p.r, p.g, p.b);
            pcl_point.x = p.x;
            pcl_point.y = p.y;
            pcl_point.z = p.z;
            pcl_cloud->points.push_back(pcl_point);
        }

        // Visualise the cloud
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(255, 255, 255);
        viewer->addPointCloud<pcl::PointXYZRGB>(pcl_cloud, "Semantic Point Cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Semantic Point Cloud");
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);  // Draw the visualizer
        }

    }

public:
    std::vector<Point> points;
};


#endif //IIQC_SEMANTICPC_H
