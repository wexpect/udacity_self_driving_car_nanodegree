/*
NOTE:

NDT（Normal Distributions Transform）算法原理与公式推导 
https://www.cnblogs.com/21207-iHome/p/8039741.html

*/



// Udacity C3 Localization
// Dec 21 2020
// Aaron Brown

using namespace std;

#include <string>
#include <sstream>
#include "helper.h"

using namespace Eigen;
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>   // TicToc

Pose pose(Point(0,0,0),Rotate(0,0,0));
Pose upose = pose;
bool matching = false;
bool update = false;

// Udacity: use the arrow keys in the same way that you did in ICP exercise to move the source point cloud around and test it with different manual poses
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{

	if (event.getKeySym() == "Right" && event.keyDown()){
		update = true;
		upose.position.x += 0.1;
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		update = true;
		upose.position.x -= 0.1;
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		update = true;
		upose.position.y += 0.1;
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		update = true;
		upose.position.y -= 0.1;
  	}
	else if (event.getKeySym() == "k" && event.keyDown()){
		update = true;
		upose.rotation.yaw += 0.02;
		while( upose.rotation.yaw > 2*pi)
			upose.rotation.yaw -= 2*pi;  
  	}
	else if (event.getKeySym() == "l" && event.keyDown()){
		update = true;
		upose.rotation.yaw -= 0.02;
		while( upose.rotation.yaw < 0)
			upose.rotation.yaw += 2*pi; 
  	}
	if (event.getKeySym() == "space" && event.keyDown()){
		matching = true;
		
  	}
	else if (event.getKeySym() == "n" && event.keyDown()){
		pose = upose;
		cout << "Set New Pose" << endl;
  	}
	
	
}

double Probability(Eigen::MatrixXd X, Eigen::MatrixXd Q, Eigen::MatrixXd S){
	// TODO: calculate the probibility of the point given mean and standard deviation
	// return 0;

	double prob = exp( -0.5 * ( (X - Q).transpose() * S.inverse() * (X - Q) )(0, 0) );
	return prob;
}

struct Cell{
	PointCloudT::Ptr cloud;
	Eigen::MatrixXd Q;
	Eigen::MatrixXd S;

	Cell(){
		PointCloudT::Ptr input(new PointCloudT);
		cloud = input;
		Q = Eigen::MatrixXd::Zero(2,1);
		S = Eigen::MatrixXd::Zero(2,2);
	}
};

struct Grid{

	// each cell is a square res x res and total grid size is (2 x width) x (2 x height)
	double res;
	int width;
	int height;

	vector<vector<Cell>> grid;

	Grid(double setRes, int setWidth, int setHeight){

		res = setRes;
		width = setWidth;
		height = setHeight;

		for(int r = 0; r < height*2; r++ ){
			vector<Cell> row;
			for(int c = 0; c < width*2; c++){
				row.push_back(Cell());
			}
			grid.push_back(row);
		}
	}

	void addPoint(PointT point){

		//cout << point.x << "," << point.y << endl;

		int c = int((point.x + width * res  ) / res);
		int r = int((point.y + height * res ) / res);

		//cout << r << "," << c << endl;

		if( (c >= 0 && c < width*2) && (r >= 0 && r < height*2) ){
			grid[r][c].cloud->points.push_back(point);
		}
	} 

	void Build(){

		for(int r = 0; r < height*2; r++ ){
			for(int c = 0; c < width*2; c++){

				PointCloudT::Ptr input = grid[r][c].cloud;
				if(input->points.size() > 2){

					// Calculate the mean
					Eigen::MatrixXd Q(2,1);
					Q << Eigen::MatrixXd::Zero(2,1);
					for(PointT point : input->points){
						Q(0,0) += point.x;
						Q(1,0) += point.y;
					}
					Q(0,0) = Q(0,0)/input->points.size();
					Q(1,0) = Q(1,0)/input->points.size();

					grid[r][c].Q = Q;

					// Calculate sigma
					Eigen::MatrixXd S(2,2);
					S << Eigen::MatrixXd::Zero(2,2);
					for(PointT point : input->points){
						Eigen::MatrixXd X(2,1);
						X(0,0) = point.x;
						X(1,0) = point.y;

						S += (X-Q) * (X-Q).transpose();
					}
					S(0,0) = S(0,0)/input->points.size();
					S(0,1) = S(0,1)/input->points.size();
					S(1,0) = S(1,0)/input->points.size();
					S(1,1) = S(1,1)/input->points.size();

					grid[r][c].S = S;
				}
				
			}
		}
	}

	Cell getCell(PointT point){
		int c = int((point.x + width * res  ) / res);
		int r = int((point.y + height * res ) / res);

		if( (c >= 0 && c < width*2) && (r >= 0 && r < height*2) ){
			return grid[r][c];
		}
		return Cell();
	}

	double Value(PointT point){

		Eigen::MatrixXd X(2,1);
		X(0,0) = point.x;
		X(1,0) = point.y;

		double value = 0;
		for(int r = 0; r < height*2; r++ ){
			for(int c = 0; c < width*2; c++){
				if(grid[r][c].cloud->points.size() > 2)
					value += Probability(X, grid[r][c].Q, grid[r][c].S );
			}
		}
		return value;
	}
};

Cell PDF(PointCloudT::Ptr input, int res, pcl::visualization::PCLVisualizer::Ptr& viewer){
	// NOTE: res, the number of areas, determines the resolution for visualization

	// TODO: Calculate the 2 x 1 matrix Q, which is the mean of the input points
	Eigen::MatrixXd Q(2, 1);
	Q << Eigen::MatrixXd::Zero(2, 1);

	for(PointT p : input->points){
		Q(0, 0) += p.x;
		Q(1, 0) += p.y;
	}
	// Q(0, 0) = Q(0, 0) / input->points.size();
	// Q(1, 0) = Q(1, 0) / input->points.size();
	Q = Q / input->points.size();


	// TODO: Calculate the 2 x 2 matrix S, which is standard deviation of the input points	
	Eigen::MatrixXd S(2, 2);
	S << Eigen::MatrixXd::Zero(2, 2);
	
	for(PointT p : input->points){
		Eigen::MatrixXd X(2, 1);		
		X(0, 0) = p.x;
		X(1, 0) = p.y;

		S += ( (X - Q) * (X - Q).transpose() );
	}
	std::cout << "S(1, 1) " << S(1, 1) << ", size " << input->points.size() << std::endl;
	S = S / input->points.size();
	std::cout << "S(1, 1) " << S(1, 1) << std::endl;


	Cell cell = Cell();
	cell.Q = Q;
	cell.S = S;
	cell.cloud = input;


	PointCloudTI::Ptr pdf(new PointCloudTI);
	for(double i = 0.0; i <= 10.0; i += 10.0/double(res)){
		for(double j = 0.0; j <= 10.0; j += 10.0/double(res)){
			Eigen::MatrixXd X(2,1);
			X(0,0) = i;
			X(1,0) = j;

			PointTI point;
			point.x = i;
			point.y = j;
			point.z = Probability(X, Q, S);
			point.intensity = point.z;

			pdf->points.push_back(point);
		}
	}
	renderPointCloudI(viewer, pdf, "pdf");

	return cell;
}

template<typename Derived>
void NewtonsMethod(PointT point, double theta, Cell cell, Eigen::MatrixBase<Derived>& g_previous, Eigen::MatrixBase<Derived>& H_previous){

	// TODO: Get the Q and S matrices from cell, invert S matrix
	Eigen::MatrixXd Q = cell.Q;
	Eigen::MatrixXd S_inv = cell.S.inverse();

	// TODO: make a 2 x 1 matrix from input point
	// QA: should be the transformed point, but here the solution does not do transform, not sure why?
	Eigen::MatrixXd X_prime(2, 1);
	X_prime(0, 0) = point.x;
	X_prime(1, 0) = point.y;

	// TODO: calculate matrix q from X and Q
	// NOTE: X should means X_prime
	Eigen::MatrixXd q = X_prime - Q;


	// TODO: calcualte the 1 x 1 exponential matrix which uses q, and S inverse
	Eigen::MatrixXd score(1, 1);
	score(0, 0) = exp( -0.5 * ( q.transpose() * S_inv * q )(0, 0) );


	// TODO: calculate the 3 2 x 1 partial derivative matrices
	// each with respect to x, y, and theta
	Eigen::MatrixXd q_p1(2, 1);
	q_p1(0, 0) = 1;
	q_p1(1, 0) = 0;

	Eigen::MatrixXd q_p2(2, 1);
	q_p2(0, 0) = 0;
	q_p2(1, 0) = 1;

	Eigen::MatrixXd q_p3(2, 1);
	q_p3(0, 0) = -point.x * sin(theta) - point.y * cos(theta);
	q_p3(1, 0) = point.x * cos(theta) - point.y * sin(theta);


	// TODO: calculate the matrix g which uses q, exponential, S inverse, and partial derivatives
	Eigen::MatrixXd g(3,1);
	g << Eigen::MatrixXd::Zero(3,1);
	g(0, 0) = (q.transpose() * S_inv * q_p1 * score)(0, 0);
	g(1, 0) = (q.transpose() * S_inv * q_p2 * score)(0, 0);
	g(2, 0) = (q.transpose() * S_inv * q_p3 * score)(0, 0);		
	
    
    // TODO: calculate the 2 x 1 second order partial derivative matrix
	Eigen::MatrixXd q_p3p3(2, 1);
	q_p3p3(0, 0) = -point.x * cos(theta) + point.y * sin(theta);
	q_p3p3(1, 0) = -point.x * sin(theta) - point.y * cos(theta);


	// TODO: calculate the matrix H which uses q, exponential, S inverse, partial derivatives, and second order partial derivative
	Eigen::MatrixXd H(3,3);
	H << Eigen::MatrixXd::Zero(3,3);
	
	H(0, 0) = ( -score * ( (q.transpose()*S_inv*q_p1)*(q.transpose()*S_inv*q_p1) - (q_p1.transpose()*S_inv*q_p1) ) )(0, 0);
	H(0, 1) = ( -score * ( (q.transpose()*S_inv*q_p1)*(q.transpose()*S_inv*q_p2) - (q_p2.transpose()*S_inv*q_p1) ) )(0, 0);
	H(0, 2) = ( -score * ( (q.transpose()*S_inv*q_p1)*(q.transpose()*S_inv*q_p3) - (q_p3.transpose()*S_inv*q_p1) ) )(0, 0);

	H(1, 0) = ( -score * ( (q.transpose()*S_inv*q_p2)*(q.transpose()*S_inv*q_p1) - (q_p1.transpose()*S_inv*q_p2) ) )(0, 0);
	H(1, 1) = ( -score * ( (q.transpose()*S_inv*q_p2)*(q.transpose()*S_inv*q_p2) - (q_p2.transpose()*S_inv*q_p2) ) )(0, 0);
	H(1, 2) = ( -score * ( (q.transpose()*S_inv*q_p2)*(q.transpose()*S_inv*q_p3) - (q_p3.transpose()*S_inv*q_p2) ) )(0, 0);

	H(2, 0) = ( -score * ( (q.transpose()*S_inv*q_p3)*(q.transpose()*S_inv*q_p1) - (q_p1.transpose()*S_inv*q_p3) ) )(0, 0);
	H(2, 1) = ( -score * ( (q.transpose()*S_inv*q_p3)*(q.transpose()*S_inv*q_p2) - (q_p2.transpose()*S_inv*q_p3) ) )(0, 0);
	// NOTE: here, idx = (2, 2), means i = j = 3
	H(2, 2) = ( -score * ( (q.transpose()*S_inv*q_p3)*(q.transpose()*S_inv*q_p3) - (q_p3.transpose()*S_inv*q_p3) - (q.transpose()*S_inv*q_p3p3) ) )(0, 0);


	H_previous += H;
	g_previous += g;
}

double Score(PointCloudT::Ptr cloud, Grid grid){
	double score = 0;
	for(PointT point:cloud->points){
		Cell cell = grid.getCell(point);
		if(cell.cloud->points.size() > 2){
			score += grid.Value(point);
		}
	}
	return score;
}

double AdjustmentScore(double alpha, Eigen::MatrixXd T, PointCloudT::Ptr source, Pose pose, Grid grid){

	T *= alpha;
	pose.position.x += T(0,0);
	pose.position.y += T(1,0);
	pose.rotation.yaw += T(2,0);
	while(pose.rotation.yaw > 2*pi)
		pose.rotation.yaw -= 2*pi;

	
	Eigen::Matrix4d transform = transform3D(pose.rotation.yaw, 0, 0, pose.position.x, pose.position.y, 0);

	PointCloudT::Ptr transformed_scan (new PointCloudT);
	pcl::transformPointCloud (*source, *transformed_scan, transform);

	double score = Score(transformed_scan, grid);

	//cout << "score would be " << score << endl;

	return score;

}

double computeStepLength(Eigen::MatrixXd T, PointCloudT::Ptr source, Pose pose, Grid grid, double currScore){
	double maxParam = max( max( T(0,0), T(1,0)), T(2,0) );
	double mlength = 1.0;
	if(maxParam > 0.2){
		mlength =  0.1/maxParam;
		T *= mlength;
	}

	double bestAlpha = 0;

	//Try smaller steps
	double alpha = 1.0;
	for(int i = 0; i < 40; i++){
		//cout << "Adjusting alpha smaller" << endl;
		double adjScore = AdjustmentScore(alpha, T, source, pose, grid);
		if( adjScore > currScore){
			bestAlpha = alpha;
			currScore = adjScore;
		}
		alpha *= .7;
	}
	if(bestAlpha == 0){
		//Try larger steps
		alpha = 2.0;
		for(int i = 0; i < 10; i++){
			//cout << "Adjusting alpha bigger" << endl;
			double adjScore = AdjustmentScore(alpha, T, source, pose, grid);
			if( adjScore > currScore){
				bestAlpha = alpha;
				currScore = adjScore;
			}
			alpha *= 2;
		}
	}

	return bestAlpha * mlength;
}

// NOTE: Positive Definite
template<typename Derived>
double PosDef(Eigen::MatrixBase<Derived>& A, double start, double increment, int maxIt){

	bool pass = false;
	int count = 0;

	A += start * Eigen::Matrix3d::Identity();
	
	while(!pass && count < maxIt){

		Eigen::LLT<Eigen::MatrixXd> lltOfA(A); // compute the Cholesky decomposition of A

    	if(lltOfA.info() == Eigen::NumericalIssue){
			A += increment * Eigen::Matrix3d::Identity ();
			count++;
    	}
		else{
			pass = true;
		}
	}

	return  start + increment * count;
}

int main(){

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("NDT Creation"));
  	viewer->setBackgroundColor (0, 0, 0);
  	viewer->addCoordinateSystem (1.0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	// Part 1: Visualize a PDF cell
	// bool part1 = true;
	bool part1 = false;

	if(part1){
		// Create Input from set of points in (x,y) range 0 - 10
		PointCloudT::Ptr input(new PointCloudT);
  		input->points.push_back(PointT(4, 4, 0));
		input->points.push_back(PointT(4, 9, 0));
		input->points.push_back(PointT(5, 5, 0));
		input->points.push_back(PointT(5, 2, 0));
		input->points.push_back(PointT(7, 5, 0));
		
		// render cell border
		renderRay(viewer, PointT(0,0,0), PointT(0,10,0), "left", Color(0,0,1));
		renderRay(viewer, PointT(0,10,0), PointT(10,10,0), "top", Color(0,0,1));
		renderRay(viewer, PointT(10,10,0), PointT(10,0,0), "right", Color(0,0,1));
		renderRay(viewer, PointT(0,0,0), PointT(10,0,0), "bottom", Color(0,0,1));

		// TODO: finish writing the PDF function to visualize the 2D gaussian
		Cell cell = PDF(input, 200, viewer);

		// Test points
		// CANDO: change test point to see how it converges
		PointT point(1,2,1); 
		
		input->points.push_back(PointT(point.x, point.y, 1.0));

		// TODO: increase the iteration count to get convergence
		for(int iteration = 0; iteration < 20; iteration++){ 
			std::cout << "\nstart iteration " << iteration << std::endl;
			std::cout << "point.x " << point.x << ", point.y " << point.y << std::endl;

			Eigen::MatrixXd g(3,1);
			g << Eigen::MatrixXd::Zero(3,1);

			Eigen::MatrixXd H(3,3);
			H << Eigen::MatrixXd::Zero(3,3);

			// TODO: finish writing the NewtonsMethod function
			// QA: seems theta is never chagned in the code?
			NewtonsMethod(point, 0, cell, g, H);
			
			// TODO: change the increment and max values to nonzero values
			// PosDef(H, 0, 0, 0); 
			PosDef(H, 0, 5, 100); 
			
			// TODO: calculate the 3 x 1 matrix T by using H inverse and g
			Eigen::MatrixXd T = -H.inverse() * g;
			std::cout << "T.p1 " << T(0, 0) << ", T.p2 " << T(1, 0)<< ", T.p3 " << T(2, 0) << std::endl;



			// QA: do not understand the logic below

			// Udacity: In part 1 of this exercise the step length was always 0.5,
			// while in part 2 its calculated in computeStepLength

			// TODO: calculate the new point by transforming point by the T matrix which is [x translation, y translation, theta rotation]
			// 	   pointT(new x, new y, 1), values below should be nonzero
			// PointT pointT(0, 0, 1); 
			PointT pointT(point.x * cos(T(2,0)) - point.y * sin(T(2,0)) + T(0,0) - point.x,  point.x * sin(T(2,0)) + point.y * cos(T(2,0)) + T(1,0) - point.y , 1);
			std::cout << "pointT.x " << pointT.x << ", pointT.y " << pointT.y << std::endl;			


			double magT = sqrt(pointT.x*pointT.x + pointT.y*pointT.y);			
			
			// CANDO: change the step size value
			double maxDelta = 0.5; 
			
			pointT.x *= maxDelta/magT;
			pointT.y *= maxDelta/magT;
			std::cout << "pointT.x " << pointT.x << ", pointT.y " << pointT.y << std::endl;			


			PointT point2(point.x+pointT.x, point.y+pointT.y, 1);
			std::cout << "point2.x " << point2.x << ", point2.y " << point2.y << std::endl;

			renderRay(viewer, point, point2, "gradient_"+to_string(iteration), Color(1,1,1));
			input->points.push_back(PointT(point2.x, point2.y, 1.0));

			point = point2;			
		}

		renderPointCloud(viewer, input, "input", Color(0,0,1));

		while (!viewer->wasStopped ())
  		{
  			viewer->spinOnce ();
  		}
	}
	// Part 2: Multiple PDF cells for target
	else{
		// Load target
		PointCloudT::Ptr target(new PointCloudT);
  		pcl::io::loadPCDFile("../target.pcd", *target);

		renderPointCloud(viewer, target, "target", Color(0,0,1));
		
		// CANDO: change grid dimension parameters
		Grid ndtGrid(3.0, 2, 2); 
		
		for(PointT point : target->points){
			ndtGrid.addPoint(point);
		}
		ndtGrid.Build();
		
		// Draw grid
		int rowc = 0;
		for(double y = -ndtGrid.height * ndtGrid.res; y <= ndtGrid.height * ndtGrid.res; y += ndtGrid.res){
			renderRay(viewer, PointT(-ndtGrid.width * ndtGrid.res,y,0), PointT(ndtGrid.width * ndtGrid.res,y,0), "grid_row_"+to_string(rowc), Color(0,0,1));
			rowc++;
		}
		int colc = 0;
		for(double x = -ndtGrid.width * ndtGrid.res; x <= ndtGrid.width * ndtGrid.res; x += ndtGrid.res){
			renderRay(viewer, PointT(x,-ndtGrid.height * ndtGrid.res,0), PointT(x,ndtGrid.height * ndtGrid.res,0), "grid_col_"+to_string(colc), Color(0,0,1));
			colc++;
		}
	
		// Draw total PDF from all cells		
		PointCloudTI::Ptr pdf(new PointCloudTI);
		int res = 10;
		for(double y = -ndtGrid.height * ndtGrid.res; y <= ndtGrid.height * ndtGrid.res; y += ndtGrid.res/double(res)){
			for(double x = -ndtGrid.width * ndtGrid.res; x <= ndtGrid.width * ndtGrid.res; x += ndtGrid.res/double(res)){
				Eigen::MatrixXd X(2,1);
				X(0,0) = x;
				X(1,0) = y;
	
				PointTI point;
				point.x = x;
				point.y = y;
				double value = ndtGrid.Value(PointT(x,y,0));
				point.z = value;
				point.intensity = value;
				if(value > 0.01)
					pdf->points.push_back(point);
			}
		}		
		renderPointCloudI(viewer, pdf, "pdf");
	
		// Load source
		PointCloudT::Ptr source(new PointCloudT);
  		pcl::io::loadPCDFile("../source.pcd", *source);
	
		renderPointCloud(viewer, source, "source", Color(1,0,0));
	
		double sourceScore = Score(source, ndtGrid);		
		viewer->addText("Score: "+to_string(sourceScore), 200, 200, 32, 1.0, 1.0, 1.0, "score",0);
	
		double currentScore = sourceScore;
	

		int iteration = 0;

		std::cout << "\nstart, matching " << matching << ", update " << update << std::endl;
  		while (!viewer->wasStopped ())
  		{
	
			if(matching){
				std::cout << "\nmatching start, matching " << matching << ", update " << update << std::endl;

				viewer->removeShape("score");
				viewer->addText("Score: "+to_string(currentScore), 200, 200, 32, 1.0, 1.0, 1.0, "score",0);
	
				Eigen::MatrixXd g(3,1);
				g << Eigen::MatrixXd::Zero(3,1);
	
				Eigen::MatrixXd H(3,3);
				H << Eigen::MatrixXd::Zero(3,3);
	
				for(PointT point : source->points){
					Cell cell = ndtGrid.getCell(point);
					if(cell.cloud->points.size() > 2){
						double theta = pose.rotation.yaw;
						double x = pose.position.x;
						double y = pose.position.y;
	
						// TODO: calculate the new point pointTran, by transforming point by x, y, and theta 
						// 	   pointTran(new x, new y, point.z), values below should be nonzero
						// PointT pointTran(0, 0, point.z);										
						PointT pointTran(point.x * cos(theta) - point.y * sin(theta) + x , point.x * sin(theta) + point.y * cos(theta) + y , point.z);

						NewtonsMethod(pointTran, theta, cell, g, H);
					}
				}
				
				// TODO: change the increment and max values to nonzero values
				// PosDef(H, 0, 0, 0); 
				PosDef(H, 0, 5, 100); 
	
				Eigen::MatrixXd T = -H.inverse()*g;
				
				// CANDO: tweek the computeStepLength function, not optimized
				double alpha = computeStepLength(T, source, pose, ndtGrid, currentScore); 
	
				T *= alpha;
	
				pose.position.x += T(0,0);
				pose.position.y += T(1,0);
				pose.rotation.yaw += T(2,0);
				while(pose.rotation.yaw > 2*pi)
					pose.rotation.yaw -= 2*pi;
	
				Eigen::Matrix4d transform = transform3D(pose.rotation.yaw, 0, 0, pose.position.x, pose.position.y, 0);
	
				PointCloudT::Ptr transformed_scan(new PointCloudT);
				pcl::transformPointCloud(*source, *transformed_scan, transform);
	
				double ndtScore = Score(transformed_scan, ndtGrid);
				
				currentScore = ndtScore;

				viewer->removeShape("nscore");
				viewer->addText("NDT Score: "+to_string(ndtScore), 200, 150, 32, 1.0, 1.0, 1.0, "nscore",0);
					
	
				viewer->removePointCloud("ndt_scan");
				renderPointCloud(viewer, transformed_scan, "ndt_scan", Color(0,1,0));
	
				iteration++;

				matching = false;
				std::cout << "matching end, matching " << matching << ", update " << update << std::endl;				
			}
			else if(update){
				std::cout << "\nupdate start, matching " << matching << ", update " << update << std::endl;				

				Eigen::Matrix4d userTransform = transform3D(upose.rotation.yaw, upose.rotation.pitch, upose.rotation.roll, upose.position.x, upose.position.y, upose.position.z);
	
				PointCloudT::Ptr transformed_scan (new PointCloudT);
  				pcl::transformPointCloud (*source, *transformed_scan, userTransform);
				viewer->removePointCloud("usource");
				renderPointCloud(viewer, transformed_scan, "usource", Color(0,1,1));
	
				
				double score = Score(transformed_scan, ndtGrid);
				viewer->removeShape("score");
				viewer->addText("Score: "+to_string(score), 200, 200, 32, 1.0, 1.0, 1.0, "score",0);
				
				update = false;
				std::cout << "update end, matching " << matching << ", update " << update << std::endl;								
			}
			
  			viewer->spinOnce ();
  		}
  	}
  	
	return 0;
}
