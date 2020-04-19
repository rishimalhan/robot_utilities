// World Manager for planners. Will contain obstacle info and collision detectors
// author: akabir@usc.edu

#ifndef WM_H
#define WM_H

#include <vector>
#include <string>
#include <cstring>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/QR>

#include <fcl/fcl.h>

#include "nabo/nabo.h"

//akabir
#include <robot_utilities/RobotCollision.hpp>
#include <robot_utilities/STLtoPCL.hpp>
#include <robot_utilities/workspace_ball.hpp>
#include <robot_utilities/workspace_ball_manager.hpp>

namespace WM{

	class WM
	{
	    private:
	        
	    public:
			Nabo::NNSearchD * nns;

	    	int slicer_xgap,slicer_ygap;

	    	bool tool_added;

	    	int env_count;
	    	int workpiece_count;

	        RCn::RobotCollision * RCp;

	        fcl::Transform3<double> the_tf;
	        fcl::Matrix3<double> R_fcl;
	        fcl::Vector3<double> t_fcl;
	        std::vector<Eigen::MatrixXd> T_all_links_fcl;
	        std::vector<fcl::Transform3<double>> TransGroup_fcl;

	        std::vector<std::pair<std::string, double>> selfDistanceList;
			std::vector<double> wpDistanceList;
	        std::vector<std::pair<std::string, double>> envDistanceList;
	        

			std::vector<std::string> WorldObjectsFilename;
	        std::vector<Eigen::MatrixXd> WorldSurfacePCL;
	        Eigen::MatrixXd SurfacePCL;
	        Eigen::MatrixXd workspace_balls;
	        int number_of_WSB;

	        int index_of_workpiece_in_WorldSurfacePCL;

	        std::vector<std::pair<std::string, std::string>> selfCollPatchPairs;
			
			Eigen::MatrixXd all_surface_xyz_n; 

			Eigen::MatrixXd M_glob;

	    	WM();
	    	~WM();
	    	
	    	//! Eigen Frame to FCL Frame
	    	fcl::Transform3<double> convertEigen2FCLFrame(const Eigen::MatrixXd& T);
	    	std::vector<fcl::Transform3<double>> convertEigen2FCLFrames(const std::vector<Eigen::MatrixXd>& link_Transforms);

	    	//! prepare world
	    	void addRobot(const std::string filepath);
	    	void addRobot(const char* filepath);


			void addTool(const std::string filepath);
	    	void addTool(const char* filepath);


			void updateTool(const std::string filepath);
			void updateTool(const char* filepath);

			
			// void updateToolPose(const Eigen::MatrixXd& tool_T); // there is bug in RC
			void removeTool();	    	

			
			void addWorkpiece(const std::string filepath);
	    	void addWorkpiece(const char* filepath);
	    	
	    	void addWorkpiece(const std::string filepath, const Eigen::MatrixXd& WP_T);
	    	void addWorkpiece(const char* filepath, const Eigen::MatrixXd& WP_T);

	    	
	    	void updateWorkpiece(const std::string filepath);	    	
	    	void updateWorkpiece(const char* filepath);	    	

	    	void updateWorkpiece(const std::string filepath, const Eigen::MatrixXd& WP_T);	    	
	    	void updateWorkpiece(const char* filepath, const Eigen::MatrixXd& WP_T);	    	

	    	
	    	void addEnvironmentObject(const std::string filepath);
	    	void addEnvironmentObject(const char* filepath);

	    	void addEnvironmentObject(const std::string filepath, const Eigen::MatrixXd& EO_T);
	    	void addEnvironmentObject(const char* filepath, const Eigen::MatrixXd& EO_T);


			void updateEnvironmentObject(int idx, const std::string filepath);
			void updateEnvironmentObject(int idx, const char* filepath);

			void updateEnvironmentObject(int idx, const std::string filepath, const Eigen::MatrixXd& EO_T);
			void updateEnvironmentObject(int idx, const char* filepath, const Eigen::MatrixXd& EO_T);


			void updateEnvironmentObjectPose(int idx, const Eigen::MatrixXd& envTransform);
			void updateEnvironmentObjectPose(const std::vector<Eigen::MatrixXd>& envTransformGroup);	

	    	
	    	void removeEnvironmentObject(int idx);

	    	
	    	Eigen::MatrixXd getSurfacePointCloud(const std::string filepath);
	    	Eigen::MatrixXd getSurfacePointCloud(const char* filepath);

	    	Eigen::MatrixXd getSurfacePointCloud(const std::string filepath, const Eigen::MatrixXd& T_world_stl);
	    	Eigen::MatrixXd getSurfacePointCloud(const char* filepath, const Eigen::MatrixXd& T_world_stl);


			void updateWorldSurfacePointCloud();
			Eigen::MatrixXd getNearestSurfacePoint(Eigen::MatrixXd p);

			void populate_workspace_balls(Eigen::MatrixXd start_node_Tee_position, Eigen::MatrixXd goal_node_Tee_position);

	    	//! boolean collision
	    	bool inCollision(const std::vector<Eigen::MatrixXd>& link_Transforms);
	    	// bool inCollision(const std::vector<fcl::Transform3<double>> link_Transforms);
	    	
	    	bool inSelfCollision(const std::vector<Eigen::MatrixXd>& link_Transforms);
	    	// bool inSelfCollision(const std::vector<fcl::Transform3<double>> link_Transforms);

	    	bool inEnvCollision(const std::vector<Eigen::MatrixXd>& link_Transforms);
	    	bool inEnvCollision(const std::vector<fcl::Transform3<double>> link_Transforms);

	    	bool inWorkpieceCollision(const std::vector<Eigen::MatrixXd>& link_Transforms);
	    	bool inWorkpieceCollision(const std::vector<fcl::Transform3<double>> link_Transforms);

	    	//! signed distance
	    	double getDistance(const std::vector<Eigen::MatrixXd>& link_Transforms);
	    	double getDistance(const std::vector<fcl::Transform3<double>> link_Transforms);

	    	void getSelfDistance(const std::vector<Eigen::MatrixXd>& link_Transforms);
	    	void getSelfDistance(const std::vector<fcl::Transform3<double>> link_Transforms);

	    	void getEnvDistance(const std::vector<Eigen::MatrixXd>& link_Transforms);
	    	void getEnvDistance(const std::vector<fcl::Transform3<double>> link_Transforms);

	    	void getWorkpieceDistance(const std::vector<Eigen::MatrixXd>& link_Transforms);
	    	void getWorkpieceDistance(const std::vector<fcl::Transform3<double>> link_Transforms);	    		    		    	

	    	//! self collision patch
	    	void prepareSelfCollisionPatch(const std::vector<Eigen::MatrixXd>& link_Transforms);
	    	bool inSelfCollisionPatch(std::string linkA, std::string linkB);
	    	bool applySelfCollisionPatch(const std::vector<Eigen::MatrixXd>& link_Transforms);



// RC->selfCollisionQuery(TransOneFrame)
// RC->envCollisionQuery(TransOneFrame)
// RC->objCollisionQuery(TransOneFrame)

// RC->selfDistanceQuery(TransOneFrame, distanceList);
// RC->envDistanceQuery(TransOneFrame, distanceList);
// RC->objDistanceQUery(TransOneFrame, objDistanceList);

	};

}
#endif        