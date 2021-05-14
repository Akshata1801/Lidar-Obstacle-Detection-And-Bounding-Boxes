// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  
  // Create the filtering object
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (filterRes, filterRes, filterRes);
  sor.filter (*cloud_filtered);
  
  //Crop box function
  
  typename pcl::PointCloud<PointT>::Ptr cropRegion (new pcl::PointCloud<PointT>); 
pcl::CropBox<PointT> region(true);
region.setMin(minPoint);
region.setMax(maxPoint);
region.setInputCloud(cloud_filtered);
region.filter(*cropRegion);
  
  std::vector<int> indices;
 //typename pcl::PointCloud<PointT>::Ptr roof (new pcl::PointCloud<PointT>); 
pcl::CropBox<PointT> roof(true);
region.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
region.setMax(Eigen::Vector4f(2.6,1.7,-4,1));
region.setInputCloud(cropRegion);
region.filter(indices);
  
  
 pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
for (int point : indices)
{
 inliers->indices.push_back(point);
  std::cout << "Found inliers" << std::endl;
}

pcl::ExtractIndices<PointT> extract;
extract.setInputCloud(cropRegion);
extract.setIndices(inliers);
extract.setNegative(true);
extract.filter(*cropRegion);  

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  
  // Create the filtering object
  pcl::ExtractIndices<PointT> extract;
  typename pcl::PointCloud<PointT>::Ptr obsCloud(new pcl::PointCloud<PointT> ());
   typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT> ());
  
  for(int index:inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);
  
  // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obsCloud);
  
//   extract.setNegative (true);
//     extract.filter (cloud_f);
   // cloud_filtered.swap (cloud_f);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
// 	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr finliers (new pcl::PointIndices ());
  //pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
//   // Create the segmentation object
//   pcl::SACSegmentation<PointT> seg;
//   // Optional
//   seg.setOptimizeCoefficients (true);
//   // Mandatory
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setMaxIterations (maxIterations);
//   seg.setDistanceThreshold (distanceThreshold);

//   // Create the filtering object
//   //pcl::ExtractIndices<pcl::PointXYZ> extract;

 
//   // While 30% of the original cloud is still there
  
//     seg.setInputCloud(cloud);
//     seg.segment (*inliers, *coefficients);
 
 std::unordered_set<int> inliersResult;
    //srand(time(NULL));

    PointT point1;
    PointT point2;
    PointT point3;

    int idx1;
    int idx2;
    int idx3;

    float a, b, c, d, dis, len;

    // For max iterations
    for (int it = 0; it < maxIterations; it++)
    {
        std::unordered_set<int> inliers;
        
        // Choose three points at random
        while (inliers.size() < 3)
        {
            inliers.insert((rand() % cloud->points.size()));
        }

        auto iter = inliers.begin();
        float x1 = cloud->points[*iter].x;
        float y1 = cloud->points[*iter].y;
        float z1 = cloud->points[*iter].z;

        ++iter;

        float x2 = cloud->points[*iter].x;
        float y2 = cloud->points[*iter].y;
        float z2 = cloud->points[*iter].z;


        ++iter;

        float x3 = cloud->points[*iter].x;
        float y3 = cloud->points[*iter].y;
        float z3 = cloud->points[*iter].z;


        // Fit a plane using the above 3 points
        a = (((y2 - y1) * (z3 - z1)) - ((z2 - z1) * (y3 - y1)));
        b = (((z2 - z1) * (x3 - x1)) - ((x2 - x1) * (z3 - z1)));
        c = (((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1)));
        d = -(a * x1 + b * y1 + c * z1);
        len = sqrt(a * a + b * b + c * c);

        // Measure distance between every point and fitted plane
        for (int pt_cnt = 0; pt_cnt < cloud->points.size(); pt_cnt++)
        {
            if (inliers.count(pt_cnt) > 0)
            {
                continue;
            }
                dis = (fabs(a * cloud->points[pt_cnt].x + b * cloud->points[pt_cnt].y + c * cloud->points[pt_cnt].z + d) / len);

                // If distance is smaller than threshold count it as inlier
                if (dis <= distanceThreshold)
                {
                    inliers.insert(pt_cnt);
                }
            
        }

        // Store the temporary buffer if the size if more than previously idenfitied points
        if (inliers.size() > inliersResult.size())
        {
            inliersResult.clear();
            inliersResult = inliers;

        }

    }

    // Segment the largest planar component from the remaining cloud
    if (inliersResult.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliersResult.count(index))
        {
            cloudInliers->points.push_back(point);
        }
        else
        {
            cloudOutliers->points.push_back(point);
        }
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  
   typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (clusterTolerance); // 2cm
  ec.setMinClusterSize (minSize);
  ec.setMaxClusterSize (maxSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);
  
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
   typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
          cloud_cluster->points.push_back(cloud->points[*pit]);
     }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
    clusters.push_back(cloud_cluster);
  }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(int indice,typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int>& cluster, std::vector<bool>& processed_points, KdTree_euclidean<PointT> *tree, float distanceTol)
{
	processed_points[indice]=true;
	cluster.push_back(indice);
	std::vector<int> nearby_points=tree->search(cloud->points[indice], distanceTol);
	for(int id:nearby_points)
	{
		if(!processed_points[id])
		{
			Proximity(id,cloud,cluster,processed_points,tree,distanceTol);
		}
	}

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree_euclidean<PointT> *tree, float distanceTol, int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed_points(cloud->points.size(),false);

	for (int i=0;i<cloud->points.size();i++)
	{
		if(!processed_points[i])
		{
			std::vector<int> cluster;
			Proximity(i,cloud,cluster,processed_points,tree,distanceTol);
            if((cluster.size() >= minSize) && (cluster.size() <= maxSize))
			    clusters.push_back(cluster);
		}
	}
 
	return clusters;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_euclidean(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  
  KdTree_euclidean<PointT> *tree(new KdTree_euclidean<PointT>);
  tree->insert_cloud (cloud);

  std::vector<std::vector<int>> cluster_indices = euclideanCluster(cloud, tree, clusterTolerance , minSize, maxSize);
  
  int j = 0;
  for (std::vector<std::vector<int>>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
  {
   typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = it->begin(); pit != it->end(); ++pit)
    {
          cloud_cluster->points.push_back(cloud->points[*pit]);
     }
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    
    clusters.push_back(cloud_cluster);
  }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}