/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // RENDER OPTIONS


    // POINT PROCESSOR
    ProcessPointClouds<pcl::PointXYZI> processor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = processor.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//    renderPointCloud(viewer, inputCloud, "inputCloud");

    float filterResolution = 0.4f;
    Eigen::Vector4f min(-10.f, -10.f, -3.f, 1.f);
    Eigen::Vector4f max(30.f, 10.f, 3.f, 1.f);
    Box filterBox;
    filterBox.x_min = min[0];
    filterBox.y_min = min[1];
    filterBox.z_min = min[2];
    filterBox.x_max = max[0];
    filterBox.y_max = max[1];
    filterBox.z_max = max[2];
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = processor.FilterCloud(inputCloud, filterResolution, min, max);
    renderPointCloud(viewer, filteredCloud, "inputCloud");

    renderBox(viewer, filterBox, 1, Color(0.f, 1.0f, 0.f), 0.2);
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    const double groundSlope = 0;
    Lidar *lidar = new Lidar(cars, groundSlope);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarScan = lidar->scan();

    // renderRays(viewer, lidar->position, lidarScan);
    // renderPointCloud(viewer, lidarScan, "LidarScan");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> processor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segResult = processor.SegmentPlane(lidarScan, 100, 0.2);
    renderPointCloud(viewer, segResult.first, "PlaneCloud", Color(0, 1, 0));
    // renderPointCloud(viewer, segResult.second, "ObstaclesCloud", Color(1, 0, 0));

    float distanceTol = 1.8;
    int minPoints = 3, maxPoints = 300;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor.Clustering(segResult.second, distanceTol, minPoints, maxPoints);

    int clusterId = 0;
    std::vector<Color> colors{Color(1, 0, 0), Color(0.5, 0.1, 0.5), Color(0, 0.0, 1.0)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "Cluster size: ";
        processor.numPoints(cluster);
        Box box = processor.BoundingBox(cluster);
        if (clusterId < colors.size())
        {
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(clusterId));
            renderBox(viewer, box, clusterId, colors.at(clusterId), 0.7);
        }
        else
        {
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId));
        }
        ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
//    simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
}
