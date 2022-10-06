// Udacity C3 Localization
// Dec 21 2020
// Aaron Brown


/*
https://www.programiz.com/cpp-programming/std-namespace#:~:text=In%20C%2B%2B%2C%20a%20namespace,in%20a%20namespace%20called%20std%20.

use the using directive to bring all the identifiers of the namespace std as if

Since using namespace std brings all the identifiers from the std namespace into 
the global namespace, this can create naming conflicts with other namespaces. 
For instance, there may be other entities with the name cout other than the 
one in the std namespace.
*/
using namespace std;

#include <string>
#include <sstream>
#include "helper.h"

#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

Eigen::Matrix4d ICP(PointCloudT::Ptr target, PointCloudT::Ptr source, Pose startingPose, int iterations){

	// std::cout << "step 1" << std::endl;

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

	//TODO: complete the ICP function and return the corrected transform
	
	// PCL ICP example
	// https://pointclouds.org/documentation/tutorials/iterative_closest_point.html

	// align source with starting pose
 	// NOTE: seems it converts the source point clouds from lidar coordinate to global coordinate
	// Transform the source to the startingPose
	cout <<"\nstartingPose: " << endl;		
	startingPose.Print();
  	Eigen::Matrix4d initTransform = transform2D(startingPose.theta, startingPose.position.x, startingPose.position.y);
	cout << "\ninitTransform: " << endl;
	print4x4Matrix(initTransform);
	Pose tmpPose = getPose(initTransform);
	cout << "tmpPose: ";
	tmpPose.Print();
	// std::cout << "step 2" << std::endl;  	  
	PointCloudT::Ptr transformSource (new PointCloudT); 
	// NOTE: A transformation on a point cloud can be done.
	// https://pointclouds.org/documentation/group__common.html#ga52d532f7f2b4d7bba78d13701d3a33d8
	// Apply an affine transform defined by an Eigen Transform.
  	pcl::transformPointCloud (*source, *transformSource, initTransform);

	// std::cout << "step 3" << std::endl;

	pcl::console::TicToc time;
  	time.tic ();
	// https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html
	// The transformation is estimated based on Singular Value Decomposition (SVD).
  	pcl::IterativeClosestPoint<PointT, PointT> icp;
  	icp.setMaximumIterations (iterations);
  	icp.setInputSource (transformSource);
  	icp.setInputTarget (target);
	// ##DIS `double`, if this value is too small then correspondences can't be made	  
	// icp.setMaxCorrespondenceDistance (##DIS);
  	PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
	// NOTE: align transformSource to target
  	icp.align (*cloud_icp);

	// std::cout << "step 4" << std::endl;	  
  	
  	if (icp.hasConverged ())
  	{
		// NOTE: icp.getFitnessScore(), the number represents the mean squared distance from each point in source to its closest point in target
  		std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
  		
		transformation_matrix = icp.getFinalTransformation ().cast<double>();
		cout << "\ntransformation_matrix: " << endl;
		print4x4Matrix(transformation_matrix);
		tmpPose = getPose(transformation_matrix);
		cout << "tmpPose: ";
		tmpPose.Print();

		// NOTE: get the overall transformation of source
  		transformation_matrix =  transformation_matrix * initTransform;
		cout << "\ntransformation_matrix * initTransform: " << endl;
		print4x4Matrix(transformation_matrix);
		tmpPose = getPose(transformation_matrix);
		cout << "tmpPose: ";
		tmpPose.Print();

  		return transformation_matrix;
  	}
  	cout << "WARNING: ICP did not converge" << endl;

	return transformation_matrix;
}

int main(){

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);

	// NOTE: global coordinste system: x points to right, y points to up, theta is from positive x axis

	// create a room
	double lowerX = -5;
	double upperX = 5;
	double lowerY = -5;
	double upperY = 5;
	vector<LineSegment> room;
	LineSegment top(0, 1, upperY, lowerX, upperX);
	room.push_back(top);
	LineSegment bottom(0, 1, lowerY, lowerX, upperX);
	room.push_back(bottom);
	LineSegment right(1, 0, upperX, lowerY, upperY);
	room.push_back(right);
	LineSegment left(1, 0, lowerX, lowerY, upperY);
	room.push_back(left);

	// create lidar
	Lidar lidar(0, 0, 0, 100, 128);

	PointCloudT::Ptr poses (new PointCloudT); 	// ground truth
	PointCloudT::Ptr locator (new PointCloudT); // estimated locations

	// starting location
	poses->points.push_back(PointT(lidar.x, lidar.y, 0));  // x, y, z
	locator->points.push_back(PointT(lidar.x, lidar.y, 0));

	// get map of room
	PointCloudT::Ptr map = lidar.scan(room);
	cout << "\nbefore move, map captured " << map->points.size() << " points" << endl;
	renderPointCloud(viewer, map, "map", Color(0,0,1)); // render map: blue



	// generate the move around the room

	// NOTE: in ICP, two scans should be close to each other. so, the magnitude and degree would not be too big.
	// So the startingPose always needs to be close to the actual position in order for ICP to work well.

	// Part 1. Localize from single step
	vector<Vect2> movement = {Vect2(0.5, pi/12)};  // 15 degree

	// vector<Vect2> movement = {Vect2(0.5, pi/12)};  // 15 degree

	// vector<Vect2> movement = {Vect2(1, pi/4)};  // 45 degree


	// Part 2. TODO: localize after several steps
	if(true){ // Change to true	
		movement.push_back(Vect2(0.8, pi/10));  // 18 degree
		movement.push_back(Vect2(1.0, pi/6));   // 30 degree

		// movement.push_back(Vect2(0.5, pi/12));  // 15 degree
		// movement.push_back(Vect2(0.5, pi/12));   // 15 degree		

		// movement.push_back(Vect2(1, pi/4));  // 45 degree
		// movement.push_back(Vect2(1, pi/4));   // 45 degree		
	}
	// Part 3. TODO: localize after randomly moving around the whole room
	if(true){ // Change to true
		srand(time(0));
		for(int i = 0; i < 10; i++){
			// NOTE: rand() between [0, RAND_MAX]
			double mag = 0.5 * ((double) rand() / (RAND_MAX)) + 0.5;  // [0.5. 1]
			double angle = pi/8 * ((double) rand() / (RAND_MAX)) + pi/8;  // [22.5 degree, 45 degree]
			movement.push_back(Vect2(mag, angle));
		}
	}


	Pose location(Point(0,0), 0);    // (x, y), theta
	PointCloudT::Ptr scan;
	int count = 0;
	for( Vect2 move : movement ){
		cout << "\n\ncount "  << count << " start" << endl;

		// execute move
		lidar.Move(move.mag, move.theta);
		poses->points.push_back(PointT(lidar.x, lidar.y, 0));   // x, y, z

		// scan the room
		scan = lidar.scan(room);
		cout << "\nscan captured " << scan->points.size() << " points" << endl;
		// for this specific render, it is in lidar coordinate system
		renderPointCloud(viewer, scan, "scan_" + to_string(count), Color(1,0,0)); // render scan: red
		 
		// perform localization
		//TODO: make the iteration count greater than zero
		// Eigen::Matrix4d transform = ICP(map, scan, location, 0); 
		Eigen::Matrix4d transform = ICP(map, scan, location, 50); 

		// NOTE: I think this is delta_x, delta_y, delta_theta, with respect to the target
		Pose estimate = getPose(transform);
		cout << "\nestimate: ";
		estimate.Print();

		// TODO: save estimate location and use it as starting pose for ICP next time
		location = estimate;

		locator->points.push_back(PointT(estimate.position.x, estimate.position.y, 0));
		
		// view transformed scan
		// TODO: perform the transformation on the scan using transform from ICP
		PointCloudT::Ptr transformedScan (new PointCloudT); 
		pcl::transformPointCloud (*scan, *transformedScan, transform);
		
		// TODO: render the correct scan
		renderPointCloud(viewer, transformedScan, "transformedScan_" + to_string(count), Color(0,1,0)); // render scan: green	

		count++;
	}

	// display ground truth poses vs estimated pose
	// renderPointCloud(viewer, poses, "poses", Color(0,1,0), 8);  // green
	// renderPath(viewer, poses, "posePath", Color(0,1,0) );

	// renderPointCloud(viewer, locator, "locator", Color(0,0,1), 6);  // blue
	// renderPath(viewer, locator, "locPath", Color(0,0,1) );

	renderPointCloud(viewer, poses, "poses", Color(1,0,0), 8);  // red
	renderPath(viewer, poses, "posePath", Color(1,0,0) );

	renderPointCloud(viewer, locator, "locator", Color(1,1,1), 6);  // white
	renderPath(viewer, locator, "locPath", Color(1,1,1) );

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce ();
	}
		
	return 0;
}


