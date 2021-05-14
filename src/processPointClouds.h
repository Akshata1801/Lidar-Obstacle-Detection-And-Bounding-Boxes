// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"



// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template<typename PointT>
struct KdTree_euclidean
{
	Node* root;

	KdTree_euclidean()
	: root(NULL)
	{}

	void insertHelper(Node *&root, uint depth, PointT point, int id)
	{

		// uint depth = 0;
		if(root == NULL)
		{
			std::vector<float> point_vector(point.data,point.data+3);
			root = new Node(point_vector,id);
		}
		else
		{
			uint cd = depth % 3;
			if(point.data[cd] < (root)->point[cd])
			{
				insertHelper(root->left,depth+1,point,id);
			}
			else
			{
				insertHelper(root->right,depth+1,point,id);
			}

	}
	}

	void insert_cloud(typename pcl::PointCloud<PointT>::Ptr cloud)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 

		for(int index=0; index < cloud->points.size();index++)
		{
			insertHelper(root,0,cloud->points[index],index);
		}
			



	}

	void searchHelper(Node *&node, uint depth, PointT target, float distanceTol, std::vector<int>* ids)
	{

		if(node!=NULL)
		{
		// std::vector<int> ids;
		if((node->point[0]<=(target.data[0]+distanceTol) && node->point[0]>=(target.data[0]-distanceTol)) && (node->point[1]<=(target.data[1]+distanceTol) && node->point[1]>=(target.data[1]-distanceTol)) && (node->point[2]<=(target.data[2]+distanceTol) && node->point[2]>=(target.data[2]-distanceTol)))
		{
			
			float x = (target.data[0]-node->point[0]);
			float y = (target.data[1]-node->point[1]);
            float z = (target.data[2]-node->point[2]);
			float distance = sqrt(x*x+y*y+z*z);
			if(distance <= distanceTol)
			{
				
				ids->push_back(node->id);
			}
		}

		uint level = depth%3;

		if((target.data[level]-distanceTol) < node->point[level])
		{
			
			searchHelper(node->left,level+1,target, distanceTol,ids);
			
		}
		if((target.data[level]+distanceTol) > node->point[level])
			searchHelper(node->right,level+1,target, distanceTol,ids);
		
		}
		


	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(root,0,target,distanceTol,&ids);
		return ids;
	}
	

};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering_euclidean(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree_euclidean<PointT> *tree, float distanceTol, int minSize, int maxSize);

    void Proximity(int indice,typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int>& cluster, std::vector<bool>& processed_points, KdTree_euclidean<PointT> *tree, float distanceTol);
    
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */