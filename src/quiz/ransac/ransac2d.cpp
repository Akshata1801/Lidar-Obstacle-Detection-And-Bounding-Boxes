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
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
  std::unordered_set<int> inliers;
  for(int i=0;i<maxIterations;i++)
  {
    std::vector<int> cor;

	for(int j =0 ;j<3;j++)
	{
		inliers.insert(rand() % cloud->points.size());
	}
    
	auto it = inliers.begin();
    int x1 = cloud->points[*it].x;
   int y1 = cloud->points[*it].y;
   int z1 = cloud->points[*it].z;
    
	++it; 

  int x2 = cloud->points[*it].x;
   int y2 = cloud->points[*it].y;
   int z2 = cloud->points[*it].z;

   ++it; 

  int x3 = cloud->points[*it].x;
   int y3 = cloud->points[*it].y;
   int z3 = cloud->points[*it].z;
    
   int a = (y2 - y1)*(z3-z1) - (z2-z1)*(y3-y1);
   int b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
   int c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
   int d = -(a*x1+b*y1+c*z1);

   for(int j=0;j<cloud->points.size();j++)
   {
	   if(inliers.count(j) > 0)
	   		continue;

		pcl::PointXYZ point = cloud->points[j];

		int x = point.x;
		int y = point.y;
		int z = point.z;

		double d = (fabs(a*x + b*y + c*z+d))/sqrt(a*a + b*b+c*c);

		if(d < distanceTol)
			inliersResult.insert(j);

   }

   if(inliers.size() > inliersResult.size())
		inliersResult = inliers;
    
  }
	
	return inliersResult;
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
  std::unordered_set<int> inliers;
  for(int i=0;i<maxIterations;i++)
  {
    std::vector<int> cor;

	for(int j =0 ;j<2;j++)
	{
		inliers.insert(rand() % cloud->points.size());
	}
    
	auto it = inliers.begin();
    int x1 = cloud->points[*it].x;
   int y1 = cloud->points[*it].y;
    
	++it; 

  int x2 = cloud->points[*it].x;
   int y2 = cloud->points[*it].y;
    
   int a = x2-x1;
   int b = y1-y2;
   int c = x1*y2 - x2*y1;

   for(int j=0;j<cloud->points.size();j++)
   {
	   if(inliers.count(j) > 0)
	   		continue;

		pcl::PointXYZ point = cloud->points[j];

		int x = point.x;
		int y = point.y;

		double d = (fabs(a*x + b*y + c))/sqrt(a*a + b*b);

		if(d < distanceTol)
			inliersResult.insert(j);

   }

   if(inliers.size() > inliersResult.size())
		inliersResult = inliers;
    
  }
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac3D(cloud, 1000, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
