/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <thread>
#include <chrono>

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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer,
               ProcessPointClouds<pcl::PointXYZI>& processor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // RENDER OPTIONS


    // POINT PROCESSOR
    Eigen::Vector4f min(-10.f, -6.0f, -3.f, 1.f);
    Eigen::Vector4f max(30.f, 6.0f, 3.f, 1.f);
    Box filterBox;
    filterBox.x_min = min[0];
    filterBox.y_min = min[1];
    filterBox.z_min = min[2];
    filterBox.x_max = max[0];
    filterBox.y_max = max[1];
    filterBox.z_max = max[2];

    Eigen::Vector4f minRoof(-1.5f, -1.5f, -1.0f, 1.0f);
    Eigen::Vector4f maxRoof(3.0f, 1.5f, 0.3f, 1.0f);
    Box filterRoofBox;
    filterRoofBox.x_min = minRoof[0];
    filterRoofBox.y_min = minRoof[1];
    filterRoofBox.z_min = minRoof[2];
    filterRoofBox.x_max = maxRoof[0];
    filterRoofBox.y_max = maxRoof[1];
    filterRoofBox.z_max = maxRoof[2];

    float filterResolution = 0.4f;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = processor.FilterCloud(inputCloud, filterResolution);
    filteredCloud = processor.FilterCloudBox(filteredCloud, min, max);
    filteredCloud = processor.FilterCloudBox(filteredCloud, minRoof, maxRoof, true);


    // Point Cloud segmentation for ground and obstacle detection
    auto segCloud = processor.SegmentPlane(filteredCloud, 100, 0.1);


    // Obstacles clustering
    float disntaceTol = 0.5;
    int minPoints = 20, maxPoints = 300;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
    obstaclesClusters = processor.Clustering(segCloud.second, disntaceTol, minPoints, maxPoints);

    std::vector<Color> colors{Color(1,0,0), Color(0.5, 0.1, 0.5), Color(0,0,1), Color(1,1,0), Color(0,1,1)};
    size_t clusterId = 0, clusterColorIdx = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : obstaclesClusters)
    {
        std::cout << "Cluster id:" << clusterId << " size:";
        processor.numPoints(cluster);
        Box box = processor.BoundingBox(cluster);
        clusterColorIdx = clusterId % colors.size();
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(clusterColorIdx));
        renderBox(viewer, box, clusterId, colors.at(clusterColorIdx), 0.3);
        ++clusterId;
    }

    // Rendering section
//    renderBox(viewer, filterBox, 100, Color(0.f, 1.0f, 0.f), 0.2);
//    renderBox(viewer, filterRoofBox, 200, Color(1.f, 1.0f, 0.f), 0.5);
//    renderPointCloud(viewer, segCloud.second, "obstaclesCloud", Color(1,0,0));
    renderPointCloud(viewer, segCloud.first, "groundCloud", Color(0,1,0));

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

    std::vector<Color> colors{Color(1,0,0), Color(0.5, 0.1, 0.5), Color(0,0,1), Color(1,1,0), Color(0,1,1)};
    size_t clusterId = 0, clusterColorIdx = 0;
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        float x = cluster->points[0].x;
        float y = cluster->points[0].y;
        float z = cluster->points[0].z;
        float distance = sqrt(x*x + y*y + z*z);
        std::cout << "Cluster id:" << clusterId << " Distance: " << distance << " size:";
        processor.numPoints(cluster);
        Box box = processor.BoundingBox(cluster);
        clusterColorIdx = clusterId % colors.size();
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(clusterColorIdx));
        renderBox(viewer, box, clusterId, colors.at(clusterColorIdx), 0.3);
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
    CameraAngle setAngle = Side;
    initCamera(setAngle, viewer);
//    simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZ>* processor (new ProcessPointClouds<pcl::PointXYZ>());
    std::vector<boost::filesystem::path> stream = processor->streamPcd("/home/oscar/dev/catkin_ws/src/ti_mmwave_rospkg/data/pcd/");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudI;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters;

    int count = 0;
    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->addCoordinateSystem();

        // Load pcd and run obstacle detection prcess
        inputCloudI = processor->loadPcd(std::string("/home/oscar/dev/catkin_ws/src/ti_mmwave_rospkg/data/pcd/") + std::to_string(count++)+std::string(".pcd"));
        renderPointCloud(viewer, inputCloudI, "input", Color(1,0,0));
        float distanceTol = 0.7;
        int minPoints = 1, maxPoints = 300;
        cloudClusters = processor->Clustering(inputCloudI, distanceTol, minPoints, maxPoints);

        std::vector<Color> colors{Color(1,0,0), Color(0.5, 0.1, 0.5), Color(0,0,1), Color(1,1,0), Color(0,1,1)};
        size_t clusterId = 0, clusterColorIdx = 0;
        for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
        {
            float x = cluster->points[0].x;
            float y = cluster->points[0].y;
            float z = cluster->points[0].z;
            float distance = sqrt(x*x + y*y + z*z);
            std::cout << "Cluster id:" << clusterId << " Distance: " << distance << " size:";
            processor->numPoints(cluster);
            Box box = processor->BoundingBox(cluster);
            clusterColorIdx = clusterId % colors.size();
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors.at(clusterColorIdx));
            renderBox(viewer, box, clusterId, colors.at(clusterColorIdx), 0.3);
            ++clusterId;
        }
        if (++streamIterator == stream.end())
        {
            streamIterator = stream.begin();
            count = 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        viewer->spinOnce();
    }
}
