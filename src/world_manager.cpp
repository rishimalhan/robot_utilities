// World Manager for planners. Will contain obstacle info and collision detectors
// author: akabir@usc.edu

#include <robot_utilities/world_manager.hpp>
#include <robot_utilities/file_rw.hpp>
namespace WM{

	WM::WM() {
		// the slicing gap needs to be adaptively selected
        slicer_xgap = 5; // mm
        slicer_ygap = 5; // mm 

		RCp = new RCn::RobotCollision();   
		// RC->init(TEST_RESOURCES_DIR"/shapes/gp8", TEST_RESOURCES_DIR"/shapes/Fan blade_Base.obj");//Fan blade_Base.obj
		// distanceTest(RC);
		// RC->init(TEST_RESOURCES_DIR"/shapes/abb", TEST_RESOURCES_DIR"/shapes/internal_gear.obj");
		// collisonTest(RC);

		T_all_links_fcl.clear();
		TransGroup_fcl.clear();
		selfDistanceList.clear();
		wpDistanceList.clear();
		envDistanceList.clear();
		WorldSurfacePCL.clear();
		WorldObjectsFilename.clear();
		selfCollPatchPairs.clear();
		number_of_WSB = 0;
		tool_added = false;
    	env_count = 0;
    	workpiece_count = 0;		
    	index_of_workpiece_in_WorldSurfacePCL = 0;

    	all_surface_xyz_n.resize(0,6);
	};

	WM::~WM(){
	};


	//! Eigen Frame to FCL Frame
	fcl::Transform3<double> WM::convertEigen2FCLFrame(const Eigen::MatrixXd& T){
			fcl::Transform3<double> temp_fcl_tf;
            t_fcl << T(0,3),T(1,3),T(2,3);
            R_fcl = T.block(0,0,3,3);
            temp_fcl_tf.linear() = R_fcl;
            temp_fcl_tf.translation() = t_fcl;
            return temp_fcl_tf;
	};

	std::vector<fcl::Transform3<double>> WM::convertEigen2FCLFrames(const std::vector<Eigen::MatrixXd>& link_Transforms){
		std::vector<fcl::Transform3<double>> fcl_tf_vec;
		fcl_tf_vec.clear();
		for (uint i=0;i<link_Transforms.size();++i){
			fcl_tf_vec.push_back(convertEigen2FCLFrame(link_Transforms[i]));
		}
		return fcl_tf_vec;
	};


	//! prepare world
	void WM::addRobot(const std::string filepath){
		addRobot(filepath.c_str());	
	};
	void WM::addRobot(const char* filepath){
		RCp->loadRobot(filepath);
		RCp->init_basic();
	};


	void WM::addTool(const std::string filepath){
		addTool(filepath.c_str());
	};

	void WM::addTool(const char* filepath){
		tool_added = true;
		RCp->addTool(filepath);
		RCp->init_basic();
	};


	void WM::updateTool(const std::string filepath){
		updateTool(filepath.c_str());
	};

	void WM::updateTool(const char* filepath){
		if(tool_added)
			RCp->updateTool(filepath);
		else
			addTool(filepath);
	};

	
	// void updateToolPose(const Eigen::MatrixXd& tool_T){
	// 	RCp->updateToolPosition(convertEigen2FCLFrame(tool_T));
	// };

	void WM::removeTool(){
		RCp->removeTool();
	};	    	


	void WM::addWorkpiece(const std::string filepath){
		addWorkpiece(filepath.c_str());
	};

	void WM::addWorkpiece(const char* filepath){
		index_of_workpiece_in_WorldSurfacePCL = WorldObjectsFilename.size();
		workpiece_count++;
		std::string dummy_str(filepath);
		WorldObjectsFilename.push_back(dummy_str);
		RCp->loadWorkpiece(filepath);
		Eigen::MatrixXd WP_T(4,4);
		WP_T = Eigen::MatrixXd::Identity(4, 4);
		WorldSurfacePCL.push_back(getSurfacePointCloud(filepath,WP_T));
		updateWorldSurfacePointCloud();
	};


	void WM::addWorkpiece(const std::string filepath, const Eigen::MatrixXd& WP_T){
		// std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$"<< std::endl;
		// std::cout << "WP: " << filepath << std::endl;
		// std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$"<< std::endl;
		addWorkpiece(filepath.c_str(), WP_T);
	};

	void WM::addWorkpiece(const char* filepath, const Eigen::MatrixXd& WP_T){
		// std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$"<< std::endl;
		// std::cout << "WP: " << filepath << std::endl;
		// std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$"<< std::endl;	
		index_of_workpiece_in_WorldSurfacePCL = WorldObjectsFilename.size();	
		workpiece_count++;
		std::string dummy_str(filepath);
		WorldObjectsFilename.push_back(dummy_str);
		RCp->loadWorkpiece(filepath, convertEigen2FCLFrame(WP_T));	
		WorldSurfacePCL.push_back(getSurfacePointCloud(filepath,WP_T));
		updateWorldSurfacePointCloud();
	};

	void WM::updateWorkpiece(const std::string filepath){
		updateWorkpiece(filepath.c_str());
	};

	void WM::updateWorkpiece(const char* filepath){
		if (workpiece_count==0)
			addWorkpiece(filepath);
		else
		{
			std::string dummy_str(filepath);
			WorldObjectsFilename[index_of_workpiece_in_WorldSurfacePCL]=dummy_str;
			RCp->loadWorkpiece(filepath);
			Eigen::MatrixXd WP_T(4,4);
			WP_T = Eigen::MatrixXd::Identity(4, 4);		
			WorldSurfacePCL[index_of_workpiece_in_WorldSurfacePCL]=getSurfacePointCloud(filepath,WP_T);
			updateWorldSurfacePointCloud();
		}
	};	    	
	
	void WM::updateWorkpiece(const std::string filepath, const Eigen::MatrixXd& WP_T){
		updateWorkpiece(filepath.c_str(), WP_T);
	};

	void WM::updateWorkpiece(const char* filepath, const Eigen::MatrixXd& WP_T){
		if (workpiece_count==0)
			addWorkpiece(filepath, WP_T);
		else
		{
			std::string dummy_str(filepath);
			WorldObjectsFilename[index_of_workpiece_in_WorldSurfacePCL]=dummy_str;
			RCp->loadWorkpiece(filepath, convertEigen2FCLFrame(WP_T));		
			WorldSurfacePCL[index_of_workpiece_in_WorldSurfacePCL]=getSurfacePointCloud(filepath,WP_T);
			updateWorldSurfacePointCloud();
		}
	};	    	


	// void WM::addEnvironmentObject(const std::string filepath){
	// 	addEnvironmentObject(filepath.c_str());
	// };

	// void WM::addEnvironmentObject(const char* filepath){
	// 	env_count++;
	// 	std::string dummy_str(filepath);
	// 	WorldObjectsFilename.push_back(dummy_str);		
	// 	RCp->addEnvironment(filepath);	
	// 	WorldSurfacePCL.push_back(getSurfacePointCloud(filepath));	
	// 	updateWorldSurfacePointCloud();
	// };


	void WM::addEnvironmentObject(const std::string filepath, const Eigen::MatrixXd& EO_T){
		// std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$"<< std::endl;
		// std::cout << "Env: " << filepath << std::endl;
		// std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$"<< std::endl;
		addEnvironmentObject(filepath.c_str(), EO_T);
	};

	void WM::addEnvironmentObject(const char* filepath, const Eigen::MatrixXd& EO_T){
		env_count++;
		std::string dummy_str(filepath);
		WorldObjectsFilename.push_back(dummy_str);
		// std::cout << "adding the env object" << std::endl;
		// std::cout << WorldObjectsFilename[env_count-1] << std::endl;
		// std::cout << "done adding the env object" << std::endl;	
		RCp->addEnvironment(filepath, convertEigen2FCLFrame(EO_T));	
		WorldSurfacePCL.push_back(getSurfacePointCloud(filepath,EO_T));
		updateWorldSurfacePointCloud();

		// std::cout << ".,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,." << std::endl;
		// for (int i=0;i<WorldObjectsFilename.size();++i)
		// {
		// 	std::cout << i << ":" << WorldObjectsFilename[i] << std::endl;
		// }
		// std::cout << filepath << std::endl;
		// std::cout << ".,.,.,.,.,.,.,.,.,.,.,.,.,.,.,.,." << std::endl;

	};


	// void WM::updateEnvironmentObject(int idx, const std::string filepath){
	// 	updateEnvironmentObject(idx, filepath.c_str());
	// };

	// void WM::updateEnvironmentObject(int idx, const char* filepath){
	// 	std::string dummy_str(filepath);
	// 	WorldObjectsFilename[idx]=dummy_str;
	// 	RCp->updateEnvironment(idx, filepath);
	// 	Eigen::MatrixXd EO_T(4,4);
	// 	EO_T = Eigen::MatrixXd::Identity(4, 4);		
	// 	WorldSurfacePCL[idx]=getSurfacePointCloud(filepath,EO_T);
	// 	updateWorldSurfacePointCloud();
	// };

	// void WM::updateEnvironmentObject(int idx, const std::string filepath, const Eigen::MatrixXd& EO_T){
	// 	updateEnvironmentObject(idx, filepath.c_str(), EO_T);
	// };

	// void WM::updateEnvironmentObject(int idx, const char* filepath, const Eigen::MatrixXd& EO_T){
	// 	std::string dummy_str(filepath);
	// 	WorldObjectsFilename[idx]=dummy_str;
	// 	RCp->updateEnvironment(idx, filepath, convertEigen2FCLFrame(EO_T));
	// 	WorldSurfacePCL[idx]=getSurfacePointCloud(filepath,EO_T);
	// 	updateWorldSurfacePointCloud();		
	// };

	void WM::updateEnvironmentObjectPose(int idx, const Eigen::MatrixXd& EO_T){
		RCp->updateEnvironmentPosition(idx, convertEigen2FCLFrame(EO_T));
		WorldSurfacePCL[idx]=getSurfacePointCloud(WorldObjectsFilename[idx],EO_T);
		updateWorldSurfacePointCloud();
	};


	void WM::updateEnvironmentObjectPose(const std::vector<Eigen::MatrixXd>& envTransformGroup){
		// std::cout << "reached wm here1" << std::endl;
		RCp->updateEnvironmentPosition(convertEigen2FCLFrames(envTransformGroup));
		// need to update surface pcl
		// std::cout << "reached wm here2" << std::endl;
		for (uint i =0;i<envTransformGroup.size();++i){
			// std::string str(WorldObjectsFilename[i]);
			// std::cout << "reached wm here I : " << WorldObjectsFilename[i] << std::endl;

			WorldSurfacePCL[i]=getSurfacePointCloud(WorldObjectsFilename[i],envTransformGroup[i]);
			// std::cout << "reached wm here J : " << i << std::endl;
			updateWorldSurfacePointCloud();
		}
		// std::cout << "reached wm here3" << std::endl;
	};	

	// void WM::removeEnvironmentObject(int idx){
	// 	env_count--;
	// 	RCp->removeEnvironment(idx);
	// 	WorldSurfacePCL.erase(WorldSurfacePCL.begin()+idx);
	// 	updateWorldSurfacePointCloud();
	// };



	Eigen::MatrixXd WM::getSurfacePointCloud(const std::string filepath){
		return getSurfacePointCloud(filepath.c_str());
	};

	Eigen::MatrixXd WM::getSurfacePointCloud(const char* filepath){
		Eigen::MatrixXd world_pts;
	    Eigen::MatrixXd world_pts_pose = STLtoPCL::gen_PCL_from_STL(filepath, slicer_xgap, slicer_ygap); // stl name, xgap, ygap
	    
	    // Eigen::MatrixXd temp_allxyzn;
	    // temp_allxyzn.resize(all_surface_xyz_n.rows()+world_pts_pose.rows(), 6);
	    // temp_allxyzn << all_surface_xyz_n, world_pts_pose;
	    // all_surface_xyz_n.resize(temp_allxyzn.rows(),temp_allxyzn.cols());
	    // all_surface_xyz_n = temp_allxyzn;

	    if(world_pts_pose.rows() <= 0)
	    	std::cout << "Something messed up with AS. STL 2 PCL failed"<<std::endl;

	    // world_pts.resize(world_pts_pose.rows(),3);
	    // world_pts = STLtoPCL::gen_positions(world_pts_pose); // rows of points
	    // if(world_pts.rows() <= 0)
	    // 	std::cout << "Something messed up. STL 2 PCL failed"<<std::endl;


	    // return world_pts;
	    return world_pts_pose;
	};	

	
	Eigen::MatrixXd WM::getSurfacePointCloud(const std::string filepath, const Eigen::MatrixXd& T_world_stl){
		return getSurfacePointCloud(filepath.c_str(), T_world_stl);
	};

	Eigen::MatrixXd WM::getSurfacePointCloud(const char* filepath, const Eigen::MatrixXd& T_world_stl){
		Eigen::MatrixXd world_pts;
		// std::cout << filepath << "\n" << slicer_xgap << "\n" <<  slicer_ygap << std::endl;
	    Eigen::MatrixXd world_pts_pose = STLtoPCL::gen_PCL_from_STL(filepath, slicer_xgap, slicer_ygap); // stl name, xgap, ygap

	    // Eigen::MatrixXd temp_allxyzn;
	    // temp_allxyzn.resize(all_surface_xyz_n.rows()+world_pts_pose.rows(), 6);
	    // temp_allxyzn << all_surface_xyz_n, world_pts_pose;
	    // all_surface_xyz_n.resize(temp_allxyzn.rows(),temp_allxyzn.cols());
	    // all_surface_xyz_n = temp_allxyzn;

	    if(world_pts_pose.rows() <= 0)
	    	std::cout << "Something messed up with AS. STL 2 PCL failed"<<std::endl;


		// // std::cout << "reached wm getSurfacePointCloud" << std::endl;
	 //    world_pts.resize(world_pts_pose.rows(),3);
	 //    // std::cout << "reached wm getSurfacePointCloud2" << std::endl;
	 //    // std::cout << world_pts_pose.rows() << "," << world_pts_pose.cols() << std::endl;
	 //    world_pts = STLtoPCL::gen_positions(world_pts_pose); // rows of points
		// // std::cout << "reached wm getSurfacePointCloud3" << std::endl;
	    
	 //    if(world_pts.rows() <= 0)
	 //    	std::cout << "Something messed up. STL 2 PCL failed"<<std::endl;

	 //    double x,y,z;
	 //    for (uint i = 0; i < world_pts.rows(); ++i){
	 //        x = T_world_stl(0,3) + T_world_stl(0,0)*world_pts(i,0) + T_world_stl(0,1)*world_pts(i,1) + T_world_stl(0,2)*world_pts(i,2);
	 //        y = T_world_stl(1,3) + T_world_stl(1,0)*world_pts(i,0) + T_world_stl(1,1)*world_pts(i,1) + T_world_stl(1,2)*world_pts(i,2);
	 //        z = T_world_stl(2,3) + T_world_stl(2,0)*world_pts(i,0) + T_world_stl(2,1)*world_pts(i,1) + T_world_stl(2,2)*world_pts(i,2);
	 //        world_pts(i,0) = x;
	 //        world_pts(i,1) = y;
	 //        world_pts(i,2) = z;
	 //    }

	    double x,y,z,nx,ny,nz;
	    for (uint i = 0; i < world_pts_pose.rows(); ++i){
	        x = T_world_stl(0,3) + T_world_stl(0,0)*world_pts_pose(i,0) + T_world_stl(0,1)*world_pts_pose(i,1) + T_world_stl(0,2)*world_pts_pose(i,2);
	        y = T_world_stl(1,3) + T_world_stl(1,0)*world_pts_pose(i,0) + T_world_stl(1,1)*world_pts_pose(i,1) + T_world_stl(1,2)*world_pts_pose(i,2);
	        z = T_world_stl(2,3) + T_world_stl(2,0)*world_pts_pose(i,0) + T_world_stl(2,1)*world_pts_pose(i,1) + T_world_stl(2,2)*world_pts_pose(i,2);
	        world_pts_pose(i,0) = x;
	        world_pts_pose(i,1) = y;
	        world_pts_pose(i,2) = z;

	        nx = T_world_stl(0,0)*world_pts_pose(i,3) + T_world_stl(0,1)*world_pts_pose(i,4) + T_world_stl(0,2)*world_pts_pose(i,5);
	        ny = T_world_stl(1,0)*world_pts_pose(i,3) + T_world_stl(1,1)*world_pts_pose(i,4) + T_world_stl(1,2)*world_pts_pose(i,5);
	        nz = T_world_stl(2,0)*world_pts_pose(i,3) + T_world_stl(2,1)*world_pts_pose(i,4) + T_world_stl(2,2)*world_pts_pose(i,5);

	        world_pts_pose(i,3) = nx;
	        world_pts_pose(i,4) = ny;
	        world_pts_pose(i,5) = nz;	        
	    }



	    // std::cout << "reached wm getSurfacePointCloud4" << std::endl;
	    
	    // return world_pts;
	    return world_pts_pose;
	};	

	void WM::updateWorldSurfacePointCloud(){
		SurfacePCL.resize(0,WorldSurfacePCL[0].cols());
		for (uint i = 0; i<WorldSurfacePCL.size();++i){
			Eigen::MatrixXd temp;
            temp.conservativeResize(SurfacePCL.rows()+WorldSurfacePCL[i].rows(), WorldSurfacePCL[i].cols());
            temp << SurfacePCL, WorldSurfacePCL[i];
			SurfacePCL.resize(temp.rows(),temp.cols());
			SurfacePCL = temp;
		}

// std::cout << SurfacePCL << std::endl;
// std::cout << "\n" << SurfacePCL.rows() << std::endl;
// file_rw::file_write("/home/ak/path_constrained_trajectory_planner/data/normals_bug_3.csv", SurfacePCL);

        M_glob.resize(3,SurfacePCL.rows());
        M_glob = SurfacePCL.block(0,0,SurfacePCL.rows(),3).transpose();
        nns = Nabo::NNSearchD::createKDTreeLinearHeap(M_glob);		
	};

	Eigen::MatrixXd WM::getNearestSurfacePoint(Eigen::MatrixXd p){
        const int K = 1;
        Eigen::VectorXi indices(K);
        Eigen::VectorXd dists2(K);
        nns->knn(p, indices, dists2, K);
        // return dists2(0);
		// std::cout << indices << std::endl;
        // std::cout << "\n" << SurfacePCL.row(indices[0]) << std::endl;
        return SurfacePCL.row(indices[0]);		
	};

    void WM::populate_workspace_balls(Eigen::MatrixXd start_node_Tee_position, Eigen::MatrixXd goal_node_Tee_position){
        // std::cout <<"$$$$$$$$$$$$$$"<<std::endl;
        // std::cout << "Surface Point Cloud: " << SurfacePCL.rows() << ", " << SurfacePCL.cols() << std::endl;
        // std::cout <<"$$$$$$$$$$$$$$"<<std::endl;

    	Eigen::MatrixXd SurfacePoints;
    	SurfacePoints.resize(SurfacePCL.rows(),3);
    	SurfacePoints = SurfacePCL.block(0,0,SurfacePCL.rows(),3);

        // WSBM::WSBM wbm(start_node_Tee_position,goal_node_Tee_position,SurfacePCL);
        WSBM::WSBM wbm(start_node_Tee_position,goal_node_Tee_position,SurfacePoints);
        wbm.create_WB_tree();
        workspace_balls.resize(4,wbm.ball_sequence_c_r.cols());
        workspace_balls = wbm.ball_sequence_c_r;
        number_of_WSB = workspace_balls.cols();
    };


	//! boolean collision
	bool WM::inCollision(const std::vector<Eigen::MatrixXd>& link_Transforms){

		// std::vector<fcl::Transform3<double>> link_tf_fcl = convertEigen2FCLFrames(link_Transforms);
		// inCollision(link_tf_fcl);
		// std::cout << " in coll 1 " << std::endl;
		// bool flag = inCollision(convertEigen2FCLFrames(link_Transforms));
		// std::cout << " in coll 1.1 " << std::endl;
		// return flag;
		// return inCollision(convertEigen2FCLFrames(link_Transforms));

		return inSelfCollision(link_Transforms) || inEnvCollision(link_Transforms)|| inWorkpieceCollision(link_Transforms);
		// return inWorkpieceCollision(link_Transforms);
		// std::cout << "# of T: " << link_Transforms.size() << std::endl;
		// return inEnvCollision(link_Transforms)|| inWorkpieceCollision(link_Transforms);
		// return false;
		
		// std::cout << "//////////////////////////"<<std::endl;
		// std::cout << link_Transforms[7]<<std::endl;
		// std::cout << "//////////////////////////"<<std::endl;

		// return inWorkpieceCollision(link_Transforms);
	};
	// bool WM::inCollision(const std::vector<fcl::Transform3<double>> link_Transforms){
	// 	// std::cout << " in coll 2 " << std::endl;
	// 	// bool flag = inSelfCollision(link_Transforms);
	// 	// std::cout << " in coll 2.1 " << std::endl;
	// 	// return flag;
	// 	return inSelfCollision(link_Transforms) || inEnvCollision(link_Transforms)|| inWorkpieceCollision(link_Transforms);
	// };
	
	// bool WM::inSelfCollision(const std::vector<Eigen::MatrixXd>& link_Transforms){
	// 	// std::cout << " self coll 1" << std::endl;
	// 	// bool flag = inSelfCollision(convertEigen2FCLFrames(link_Transforms));
	// 	// std::cout << " self coll 1.1" << std::endl;
	// 	// return flag;
	// 	// return inSelfCollision(convertEigen2FCLFrames(link_Transforms));

	// 	return applySelfCollisionPatch(link_Transforms);
	// };

	bool WM::inSelfCollision(const std::vector<Eigen::MatrixXd>& link_Transforms){
		// std::cout << " self coll 1" << std::endl;
		// bool flag = inSelfCollision(convertEigen2FCLFrames(link_Transforms));
		// std::cout << " self coll 1.1" << std::endl;
		// return flag;
		// return inSelfCollision(convertEigen2FCLFrames(link_Transforms));
		return RCp->selfCollisionQuery(convertEigen2FCLFrames(link_Transforms));
		// return applySelfCollisionPatch(link_Transforms);
	};


	// bool WM::inSelfCollision(const std::vector<fcl::Transform3<double>> link_Transforms){
	// 	// std::cout << " self coll 2" << std::endl;
	// 	// std::cout << " self coll count: " << RCp->selfCollisionQuery(link_Transforms) << std::endl;
	// 	// bool flag = RCp->selfCollisionQuery(link_Transforms);
	// 	// std::cout << " self coll 2.1" << std::endl;
	// 	// return flag;


	// 	bool flag = RCp->selfCollisionQuery(link_Transforms);

	// 	// std::cout << "bool coll pairs: " << std::endl;
	// 	flag = false;
	// 	for (uint i =0; i<RCp->selfCollisionPair[0].size(); ++i){
						
	// 		if(RCp->selfCollisionPair[0][i].first == "Link4" && RCp->selfCollisionPair[0][i].second == "Link6")
	// 			continue;
	// 		if(RCp->selfCollisionPair[0][i].first == "Link6" && RCp->selfCollisionPair[0][i].second == "Link4")
	// 			continue;			
	// 		// std::cout << RCp->selfCollisionPair[0][i].first << ", " << RCp->selfCollisionPair[0][i].second << std::endl;
	// 		flag = true;
	// 	}

	// 	// if(flag){
	// 		// std::cout << "Self coll: " << flag << std::endl;
			
	// 	// }
		

	// 	return flag;
	// };

	bool WM::inEnvCollision(const std::vector<Eigen::MatrixXd>& link_Transforms){
		// std::cout << "debug env" << std::endl;
		if(env_count<=0)
			return false;
		return inEnvCollision(convertEigen2FCLFrames(link_Transforms));
	};
	bool WM::inEnvCollision(const std::vector<fcl::Transform3<double>> link_Transforms){
		if(env_count<=0)
			return false;		
		// std::cout << " env coll " << std::endl;
		// std::cout << "Env coll: " << RCp->envCollisionQuery(link_Transforms) << std::endl;
		return RCp->envCollisionQuery(link_Transforms);
	};

	bool WM::inWorkpieceCollision(const std::vector<Eigen::MatrixXd>& link_Transforms){
		// std::cout << "debug wp" << std::endl;
		if(workpiece_count<=0)
			return false;		
		return inWorkpieceCollision(convertEigen2FCLFrames(link_Transforms));
	};
	bool WM::inWorkpieceCollision(const std::vector<fcl::Transform3<double>> link_Transforms){
		if(workpiece_count<=0)
			return false;		
		// std::cout << " wp coll " << std::endl;
		// std::cout << "WP coll: " << RCp->objCollisionQuery(link_Transforms) << std::endl;
		// RCp->objCollisionQuery(link_Transforms);
		// for (int i = 0;i<RCp->objCollisionPair[0].size();++i){
		// 	std::cout << "1: " << RCp->objCollisionPair[0][i].first	<<", 2: "<<RCp->objCollisionPair[0][i].second<<std::endl;	
		// }
		return RCp->objCollisionQuery(link_Transforms);
	};


	//! signed distance
	double WM::getDistance(const std::vector<Eigen::MatrixXd>& link_Transforms){
		return getDistance( convertEigen2FCLFrames(link_Transforms) );
	};
	double WM::getDistance(const std::vector<fcl::Transform3<double>> link_Transforms){
		// std::cout << "bool coll pairs: " << std::endl;
		// for (uint i =0; i<RCp->selfCollisionPair[0].size(); ++i){
		// 	std::cout << RCp->selfCollisionPair[0][i].first << ", " << RCp->selfCollisionPair[0][i].second << std::endl;			
		// }
		

		double dist = 0;
		double min_d = 10000000000000;
		// getSelfDistance(link_Transforms);
		getEnvDistance(link_Transforms);
		getWorkpieceDistance(link_Transforms);

		for(int i=0; i<wpDistanceList.size(); ++i)
			std::cout<< wpDistanceList[i] << " , ";
		std::cout<< "\n";

		for(int i = 0; i < RCp->robotLinks.size(); ++i)
		{
			// if(selfDistanceList.size()){
			// 	if(selfDistanceList[i].second<0){


			// 		// if( !(i == 4 && selfDistanceList[i].first== "Link6") && !(i == 6 && selfDistanceList[i].first == "Link4") )	

			// 		if(!inSelfCollisionPatch("Link"+to_string(i), selfDistanceList[i].first))
			// 		{
			// 			// printf("link %i - ", i);
			// 			// printf("%s \n", selfDistanceList[i].first.c_str());

			// 			// std::cout << selfDistanceList[i].first << std::endl;
			// 			dist = dist + selfDistanceList[i].second;
			// 			if(min_d>selfDistanceList[i].second){
			// 				min_d = selfDistanceList[i].second;
			// 			}							
			// 		}
			// 	}				
			// }
			if(envDistanceList.size()){
				if(envDistanceList[i].second<0){
					// std::cout << "link: " << i << ", env dist: " << envDistanceList[i] << std::endl;
					dist = dist + envDistanceList[i].second;
				}				
				if(min_d>envDistanceList[i].second){
					min_d = envDistanceList[i].second;
				}				
			}				

			if(wpDistanceList.size()){
				if(wpDistanceList[i]<0){
					// std::cout << "link: " << i << ", wp dist: " << wpDistanceList[i] << std::endl;
					dist = dist + wpDistanceList[i];
				}
				if(min_d>wpDistanceList[i]){
					min_d = wpDistanceList[i];							
				}				
			}

		}

		// std::cout << "dist: " << dist << ", min_d: " << min_d<<std::endl;;

		if(dist>=0)
			return min_d;
		return dist;
	};

	void WM::getSelfDistance(const std::vector<Eigen::MatrixXd>& link_Transforms){
		getSelfDistance(convertEigen2FCLFrames(link_Transforms));
	};
	void WM::getSelfDistance(const std::vector<fcl::Transform3<double>> link_Transforms){
		selfDistanceList.clear();
		RCp->selfDistanceQuery(link_Transforms, selfDistanceList);
	};

	void WM::getEnvDistance(const std::vector<Eigen::MatrixXd>& link_Transforms){
		if(env_count<=0)
			return;
		getEnvDistance(convertEigen2FCLFrames(link_Transforms));
	};
	void WM::getEnvDistance(const std::vector<fcl::Transform3<double>> link_Transforms){
		if(env_count<=0)
			return;		
		envDistanceList.clear();
		RCp->envDistanceQuery(link_Transforms, envDistanceList);
	};

	void WM::getWorkpieceDistance(const std::vector<Eigen::MatrixXd>& link_Transforms){
		if(workpiece_count<=0)
			return;		
		getWorkpieceDistance(convertEigen2FCLFrames(link_Transforms));
	};
	void WM::getWorkpieceDistance(const std::vector<fcl::Transform3<double>> link_Transforms){
		if(workpiece_count<=0)
			return;				
		wpDistanceList.clear();
		RCp->objDistanceQUery(link_Transforms, wpDistanceList);
	};

	//! self collision patch
	void WM::prepareSelfCollisionPatch(const std::vector<Eigen::MatrixXd>& link_Transforms){
		// std::cout << "pp 1" << std::endl;
		// std::vector<fcl::Transform3<double>> link_Transforms_fcl = convertEigen2FCLFrames(link_Transforms);
		selfCollPatchPairs.clear();
		RCp->selfCollisionQuery(convertEigen2FCLFrames(link_Transforms));
		// std::cout << "pp 2 " << std::endl;

		// std::cout << "pp 3 " << RCp->selfCollisionPair[0].size() << std::endl;

		for (int i = 0; i<RCp->selfCollisionPair[0].size(); ++i){
			// std::cout << "pp 3" << std::endl;
			// std::cout << "Self Coll Patch: " << RCp->selfCollisionPair[0][i].first	<<", "<<RCp->selfCollisionPair[0][i].second<<std::endl;	
			// selfCollPatchPairs.push_back(make_pair(RCp->selfCollisionPair[0][i].first,RCp->selfCollisionPair[0][i].second));
			selfCollPatchPairs.push_back(RCp->selfCollisionPair[0][i]);
		}		
	};

	bool WM::inSelfCollisionPatch(std::string linkA, std::string linkB){
		bool temp = false;		
		for (uint j = 0; j<selfCollPatchPairs.size();++j){
			if(	selfCollPatchPairs[j].first == linkA && 
				selfCollPatchPairs[j].second == linkB )
			{
				temp = true;
			}
			// else{
			// 	std::cout << std::endl;
			// 	std::cout << "Self Coll: " << linkA	<<", "<<linkB<<std::endl;	
			// 	std::cout << "Self Coll: " << selfCollPatchPairs[j].first	<<", "<<selfCollPatchPairs[j].second<<std::endl;	
			// 	std::cout << std::endl;
			// }
		}		
		if(!temp)
			// std::cout << "Self Coll: " << linkA	<<", "<<linkB<<std::endl;
		return temp;
	};

	bool WM::applySelfCollisionPatch(const std::vector<Eigen::MatrixXd>& link_Transforms){
		bool flag = false;
		// std::cout << "bug 1" << std::endl;
		RCp->objCollisionQuery(convertEigen2FCLFrames(link_Transforms));
		// std::cout << "bug 2" << std::endl;
		for (int i = 0;i<RCp->selfCollisionPair[0].size();++i){
			if(!inSelfCollisionPatch(RCp->selfCollisionPair[0][i].first, RCp->selfCollisionPair[0][i].second)){
				flag = true;
				return flag;
			}
		}	
		// std::cout << "bug 3" << std::endl;
		return flag;
	};


}


