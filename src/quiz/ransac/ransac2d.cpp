/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	auto generateRandNum = [=](int max, int min) { return rand() % (max - min + 1) + min; };
	srand(time(NULL));

	// TODO: Fill in this function
	// For max iterations
	while (--maxIterations)
	{
		// Randomly sample subset and fit line
		pcl::PointXYZ point1, point2, point3;
		int randNum1, randNum2, randNum3;
		do
		{
			randNum1 = generateRandNum(0, cloud->points.size() - 1);
			randNum2 = generateRandNum(0, cloud->points.size() - 1);
			randNum3 = generateRandNum(0, cloud->points.size() - 1);
			point1 = cloud->points[randNum1];
			point2 = cloud->points[randNum2];
			point3 = cloud->points[randNum3];
		} while (randNum1 == randNum2 == randNum3);

		double x1 = cloud->points[randNum1].x;
		double y1 = cloud->points[randNum1].y;
		double z1 = cloud->points[randNum1].z;
		double x2 = cloud->points[randNum2].x;
		double y2 = cloud->points[randNum2].y;
		double z2 = cloud->points[randNum2].z;
		double x3 = cloud->points[randNum3].x;
		double y3 = cloud->points[randNum3].y;
		double z3 = cloud->points[randNum3].z;

		Vect3 v1{x2 - x1, y2 - y1, z2 - z1};
		Vect3 v2{x3 - x1, y3 - y1, z3 - z1};
		// Cross product v1Xv2
		Vect3 v1xv2{(y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1), (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1), (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)};

		double i = v1xv2.x;
		double j = v1xv2.y;
		double k = v1xv2.z;
		double A = i;
		double B = j;
		double C = k;
		double D = -1 * (i * x1 + j * y1 + k * z1);

		// Measure distance between every point and fitted line
		std::unordered_set<int> inliers;
		for (size_t i = 0; i < cloud->points.size(); ++i)
		{
			pcl::PointXYZ point = cloud->points[i];
			double dist = fabs(A * point.x + B * point.y + C * point.z + D) / sqrt(A * A + B * B + C * C);
			// If distance is smaller than threshold count it as inlier
			if (dist <= distanceTol)
				inliers.insert(i);
		}
		if (inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "RANSAC took: " << elapsedTime.count() << " miliseconds\n";
	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}

int main()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	// Render 2D point cloud with inliers and outliers
	if (inliers.size())
	{
		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
	}
	else
	{
		renderPointCloud(viewer, cloud, "data");
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}
