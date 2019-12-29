// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
void ProcessPointClouds<PointT>::proximity(int index, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[index] = true;
	cluster.push_back(index);

	// we gonna see which points nearest by to this index 
	std::vector<int> nearest = tree->search(points[index], distanceTol);

	for(int id : nearest) 
	{
		if(!processed[id])
			proximity(id, points, cluster, processed, tree, distanceTol);
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(), false);

	for(int i=0 ; i<points.size(); ++i)
	{
		if(processed[i])
		{
			continue;
		}

		std::vector<int> cluster;
		proximity(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
	}
 
	return clusters;

}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);  
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered); 

    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> regionBox(true);
    regionBox.setMin(minPoint);
    regionBox.setMax(maxPoint);
    regionBox.setInputCloud(cloud_filtered);
    regionBox.filter(*cloud_region);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_region);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_region);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    // Passing inliers in planeCloud
    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    //  Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    
    // // create a segment object
    // pcl::SACSegmentation<PointT> seg;
    // // pcl typr point indices and will be using later 
    // // to seprate pointCloud into two pices
    // pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // // define coeffecient of our model and we will define model typr tp SACMODEL_PLANE later
    // // and these coeffecients eill actually be defining what this plane is
    // pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);

    // // Segment the largest planar component from the input cloud
    // seg.setInputCloud(cloud);
    // // generate inliers and coefficients from the cloud
    // seg.segment(*inliers, *coefficients);
    // // if we didn't find any model that can fit this data
    // if(inliers->indices.size() == 0)
    // {
    //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    // }

    /*  ///////////////////////////
        IMPLEMENTATION FROM SCRATCH
        ///////////////////////////
    */
    std::unordered_set<int> inliersResult;
	srand(time(NULL));
	std::unordered_set<int> inliers;
	int numPoints = cloud->points.size();
	float x1, y1, z1, x2, y2, z2, x3, y3, z3;
	float a, b, c, d;
	std::vector<float> v1, v2;

	while(maxIterations--){		
		// Randomly pick 3 points 		
		while(inliers.size() < 3)
			inliers.insert(rand() % numPoints);
		
		auto ptr = std::begin(inliers);
		x1 = cloud->points[*ptr].x;
		y1 = cloud->points[*ptr].y;
		z1 = cloud->points[*ptr].z;
		ptr++;
		x2 = cloud->points[*ptr].x;
		y2 = cloud->points[*ptr].y;
		z2 = cloud->points[*ptr].z;
		ptr++;
		x3 = cloud->points[*ptr].x;
		y3 = cloud->points[*ptr].y;
		z3 = cloud->points[*ptr].z;		


		a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
		b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
		c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
		d = -(a * x1 + b * y1 + c * z1);

		for(int index=0; index < numPoints; index++){
			// if the point in our sampled line two points 
			if(inliers.count(index) > 0)
				continue;
			
			// pcl::PointXYZ point = cloud->points[index];
			float x4 = cloud->points[index].x;
			float y4 = cloud->points[index].y;
			float z4 = cloud->points[index].z;

			float distance = std::abs(a*x4 + b*y4 + c*z4 + d) / std::sqrt(a*a + b*b + c*c);

			if(distance <= distanceThreshold)
				inliers.insert(index);
		}

		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
		
		inliers.clear();
	}
	
	if(inliersResult.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    typename pcl::PointIndices::Ptr cloudInliers (new pcl::PointIndices());
	for (int point : inliersResult) {
		cloudInliers->indices.push_back(point);
	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(cloudInliers,cloud);

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Creating the KdTree object for the search method of the extraction
    // typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud (cloud);

    // std::vector<pcl::PointIndices> clusterIndices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // // Any points within that distance tolerance will group together
    // ec.setClusterTolerance (clusterTolerance); 
    // // It also has min and max arguments for the number of points to represent as clusters.
    // ec.setMinClusterSize (minSize);
    // ec.setMaxClusterSize (maxSize);
    // // the tree objrct is created and built using the input cloud points, which in this case are going to be the obstacle cloud points.
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (cloud);
    // ec.extract (clusterIndices);

    // // So next I will go through cluster indices and create some point clouds going to be a different clusters
    // for( pcl::PointIndices getIndices : clusterIndices)
    // {
    //     typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

    //     for(int index : getIndices.indices)
    //         cloudCluster->points.push_back (cloud->points[index]);
        
    //     cloudCluster->width = cloudCluster->points.size();
    //     cloudCluster->height = 1;
    //     cloudCluster->is_dense = true;

    //     clusters.push_back(cloudCluster);
    // }

    /*  ///////////////////////////
        IMPLEMENTATION FROM SCRATCH
        ///////////////////////////
    */
    std::vector<std::vector<float>> points;
    KdTree* tree = new KdTree(3);
  
    for (int i=0; i<cloud->points.size(); i++){
		auto point = cloud->points[i];
		tree->insert({point.x, point.y, point.z}, i);
		points.push_back({point.x, point.y, point.z});
	}

    std::vector<std::vector<int>> indices = euclideanCluster(points, tree, clusterTolerance);
    
	for (std::vector<int> v : indices) {
		typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
		if (v.size() >= minSize && v.size() <= maxSize) {
			for (int clustInd : v) {
				cluster->points.push_back(cloud->points[clustInd]);
			}
			clusters.push_back(cluster);
		}
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