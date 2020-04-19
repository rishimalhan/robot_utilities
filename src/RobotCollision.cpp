#include <robot_utilities/RobotCollision.hpp>


struct CollisionData
{
  CollisionData()
  {
    done = false;
  }
  std::set<std::pair<fcl::CollisionObject<double>*, fcl::CollisionObject<double>*>> collidePairs;
  fcl::CollisionRequest<double> request;
  fcl::CollisionResult<double> result;
  bool addCollidePair(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2)
  {
  	auto search = collidePairs.find(std::make_pair(o1, o2));
    if (search != collidePairs.end())
      return false;
    collidePairs.emplace(o1, o2);
    return true;
  }
  bool done;
};

bool CollisionFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_)
{
	auto* cdata = static_cast<CollisionData*>(cdata_);
	const auto& request = cdata->request;
	auto& result = cdata->result;
	fcl::collide(o1, o2, request, result);
	if(result.isCollision())
	{
		cdata->addCollidePair(o1, o2);
		result.clear();
	}
	return false;
}

struct DistanceData
{
	DistanceData()
	{
		done = false;
	}
	std::set<std::pair<std::pair<fcl::CollisionObject<double>*, fcl::CollisionObject<double>*>, double>> closePairs;
	fcl::DistanceRequest<double> request = fcl::DistanceRequest<double>(true, true, 0, 0, 1e-8, fcl::GJKSolverType::GST_INDEP);
	fcl::DistanceResult<double> result;
	bool addClosePair(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, double min_distance)
	{
	  	auto search = closePairs.find(std::make_pair(std::make_pair(o1, o2), min_distance));
    	if (search != closePairs.end())
      		return false;
    	closePairs.emplace(std::make_pair(o1, o2), min_distance);
    	return true;
	}
	bool done;
};

bool DistanceFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_, double & dist)
{
	auto* cdata = static_cast<DistanceData*>(cdata_);
	const auto& request = cdata->request;
	auto& result = cdata->result;
	fcl::distance(o1, o2, request, result);
	dist = result.min_distance;
	cdata->addClosePair(o1, o2, dist);
	result.clear();
	dist = 0.01; //if dist <= 0, this code will cease to proceed
	return false;
}

struct DistanceOnePair
{
	DistanceOnePair()
	{
		done = false;
	}
	bool done;
	std::pair<fcl::CollisionObject<double>*, double> ObjectDistancePair;
	fcl::DistanceRequest<double> request = fcl::DistanceRequest<double>(true, true, 0, 0, 1e-8, fcl::GJKSolverType::GST_INDEP);
	fcl::DistanceResult<double> result;
	double min_distance;
	bool makePair(fcl::CollisionObject<double>* obj, double dist)
	{
		ObjectDistancePair = std::make_pair(obj, dist);
		min_distance = dist;
		return true;
	}
};

bool OnePairDistanceFunction(fcl::CollisionObject<double>* o1, fcl::CollisionObject<double>* o2, void* cdata_)
{
	auto* cdata = static_cast<DistanceOnePair*>(cdata_);
	const auto& request = cdata->request;
	auto& result = cdata->result;
	fcl::distance(o1, o2, request, result);
	double dist = result.min_distance;
	if (dist <= cdata->min_distance)
	{
		cdata->makePair(o2, dist);
	}
	result.clear();
}




namespace RCn{


typedef fcl::BVHModel<fcl::OBBRSS<double>> Model;

RobotCollision::RobotCollision(void)
{

}

RobotCollision::~RobotCollision(void)
{
	//do nothing
}

void RobotCollision::loadRobot(const char* filepath)
{
	printf("initializing robot..............\n");
	printf("=========================================================\n");
	time_t start, stop;
	//////////////////////////////////search file///////////////////////////
	start = clock();
	this->robotLinks.clear();
	this->environment.clear();
	DIR *pDir;
	struct dirent* ptr;
	std::string spath(filepath);
	if(!(pDir = opendir(spath.c_str())))
	{
		printf("Folder does not Exist\n");
		return;
	}
	std::vector<std::string> filenames, modelNames, envNames;
	while((ptr = readdir(pDir)) != 0)
	{
		if(strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
		{
			filenames.push_back(ptr->d_name);
		}
	}
	closedir(pDir);
	for(int i = 0; i < filenames.size(); ++i)
	{
		if (((filenames[i].find(".stl")!= std::string::npos)
			||(filenames[i].find(".STL")!= std::string::npos)
			||(filenames[i].find(".obj")!= std::string::npos)
			||(filenames[i].find(".OBJ")!= std::string::npos))
			&&(filenames[i].find("link")!= std::string::npos))
		{
			modelNames.push_back(spath + "/" + filenames[i]);
		}
		else if (((filenames[i].find(".stl")!= std::string::npos)
			||(filenames[i].find(".STL")!= std::string::npos)
			||(filenames[i].find(".obj")!= std::string::npos)
			||(filenames[i].find(".OBJ")!= std::string::npos))
			&&(filenames[i].find("environment")!= std::string::npos))
		{
			envNames.push_back(spath + "/" + filenames[i]);
		}
	}
	std::sort(modelNames.begin(), modelNames.end());
	if (envNames.size() > 1)
		std::sort(envNames.begin(), modelNames.end());
	stop = clock();
	double cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
	printf("searching files costs %fs\n", cost_time);
	printf("=========================================================\n");
	//////////////////////////read file//////////////////////////////
	start = clock();
	for (int i = 0; i < modelNames.size(); ++i)
	{
		std::vector<fcl::Vector3<double>> p;
		std::vector<fcl::Triangle> t;
		std::shared_ptr<Model> geom = std::make_shared<Model>();
		ReadMesh::loadModel(modelNames[i].c_str(), p, t);
		geom->beginModel();
		geom->addSubModel(p, t);
		geom->endModel();
		fcl::CollisionObject<double>* link = new fcl::CollisionObject<double>(geom);
		this->robotLinks.push_back(link);
		printf("link%i has %li points and %li triangles\n", i, p.size(), t.size());
	}
	printf("=========================================================\n");
	for (int i = 0; i < envNames.size(); ++i)
	{
		std::vector<fcl::Vector3<double>> p;
		std::vector<fcl::Triangle> t;
		std::shared_ptr<Model> geom = std::make_shared<Model>();
		ReadMesh::loadModel(envNames[i].c_str(), p, t);
		geom->beginModel();
		geom->addSubModel(p, t);
		geom->endModel();
		fcl::CollisionObject<double>* env = new fcl::CollisionObject<double>(geom);
		this->environment.push_back(env);
		printf("environment%i has %li points and %li triangles\n", i, p.size(), t.size());
	}
	printf("=========================================================\n");
	stop = clock();
	cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
	printf("reading files costs %fs\n", cost_time);
	printf("=========================================================\n");
	this->linkNum = this->robotLinks.size();
	// this->envNum = this->environment.size();

	/////////////////attach tag/////////////////////////
	for (int i = 0; i < this->linkNum; ++i)
	{
		std::ostringstream oss;
		oss<<"Link"<<i;
		this->linkLabel.push_back(oss.str());	
	}
	for (int i = 0; i < this->linkNum; ++i)
		this->robotLinks[i]->setUserData(&this->linkLabel[i]);

	// for (int i = 0; i < this->envNum; ++i)
	// {
	// 	std::ostringstream oss;
	// 	oss<<"Environment"<<i;
	// 	this->envLabel.push_back(oss.str());
	// }	
	// for (int i = 0; i < this->envNum; ++i)
	// 	this->environment[i]->setUserData(&this->envLabel[i]);
	return;
}

void RobotCollision::loadWorkpiece(const char* filepath)
{
	time_t start, stop;
	start = clock();
	std::vector<fcl::Vector3<double>> p;
	std::vector<fcl::Triangle> t;
	std::shared_ptr<Model> geom = std::make_shared<Model>();
	ReadMesh::loadModel(filepath, p, t);
	geom->beginModel();
	geom->addSubModel(p, t);
	geom->endModel();
	this->workpiece = new fcl::CollisionObject<double>(geom);
	this->workpiece->setUserData(&this->objectLabel[0]);
	stop = clock();
	double cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
	printf("loading workpiece costs %fs\n", cost_time);
	printf("workpiece has %li points and %li triangles\n", p.size(), t.size());
	printf("=========================================================\n");
	return;
}

void RobotCollision::loadWorkpiece(const char* filepath, const fcl::Transform3<double> WP_transform)
{
	time_t start, stop;
	start = clock();
	std::vector<fcl::Vector3<double>> p;
	std::vector<fcl::Triangle> t;
	std::shared_ptr<Model> geom = std::make_shared<Model>();
	ReadMesh::loadModel(filepath, p, t);
	geom->beginModel();
	geom->addSubModel(p, t);
	geom->endModel();
	this->workpiece = new fcl::CollisionObject<double>(geom, WP_transform);
	this->workpiece->setUserData(&this->objectLabel[0]);
	stop = clock();
	double cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
	printf("loading workpiece costs %fs\n", cost_time);
	printf("workpiece has %li points and %li triangles\n", p.size(), t.size());
	printf("=========================================================\n");	
	return;
};

void RobotCollision::init_basic()
{
	time_t start, stop;
	printf("initializing collision.........\n");
	this->Manager = new fcl::NaiveCollisionManager<double>();
	start = clock();
	this->Manager->registerObjects(this->robotLinks);
	this->Manager->setup();
	stop = clock();
	double cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
	printf("initializing collision costs %fs\n", cost_time);
}

void RobotCollision::init(const char* robotPath, const char* workpiecePath)
{
	time_t start, stop;
	this->loadRobot(robotPath);
	this->loadWorkpiece(workpiecePath);
	printf("initializing collision.........\n");
	this->Manager = new fcl::NaiveCollisionManager<double>();
	start = clock();
	this->Manager->registerObjects(this->robotLinks);
	this->Manager->setup();
	stop = clock();
	double cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
	printf("initializing collision costs %fs\n", cost_time);
}


void RobotCollision::init(const char* robotPath, const char* workpiecePath, const fcl::Transform3<double> WP_transform)
{
	time_t start, stop;
	this->loadRobot(robotPath);
	this->loadWorkpiece(workpiecePath, WP_transform);
	printf("initializing collision.........\n");
	this->Manager = new fcl::NaiveCollisionManager<double>();
	start = clock();
	this->Manager->registerObjects(this->robotLinks);
	this->Manager->setup();
	stop = clock();
	double cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
	printf("initializing collision costs %fs\n", cost_time);	
};


// void RobotCollision::collisionQuery(const std::vector<std::vector<fcl::Transform3<double>>>& TransGroup)
// {
// 	this->selfCollisionPair.clear();
// 	this->envCollisionPair.clear();
// 	this->objCollisionPair.clear();
// 	std::vector<std::pair<std::string, std::string>> sCP, eCP, oCP;
// 	printf("checking transforms......\n");
// 	int link_data = TransGroup.size();
// 	// if (link_data != (this->linkNum + this->toolFlag))
// 	// {
// 	// 	std::cerr<<"the data are inconsistant with robot link number\n";
// 	// 	return;
// 	// }
// 	int frame_data = TransGroup[0].size();
// 	int frameNum = 0;
// 	for (int i = 0; i < link_data; ++i)
// 	{
// 		if (TransGroup[i].size() != frame_data)
// 		{
// 			std::cerr<<"the frame number of link "<<i<<"is inconsistant with others\n";
// 			return;
// 		}
// 	}
// 	frameNum = frame_data;
// 	time_t start, stop;
// 	printf("loading transforms......\n");
// 	for(int j = 0; j < frameNum; ++j)
// 	{
// 		start = clock();
// 		for (int i = 0; i < this->robotLinks.size(); ++i)
// 		{
// 			this->robotLinks[i]->setTransform(TransGroup[i][j]);
// 			this->robotLinks[i]->computeAABB();
// 		}
// 		this->Manager->update();
// 		stop = clock();
// 		double cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
// 		/////////////self collision check////////////////
// 		struct CollisionData self_data, env_data, obj_data;
// 		start = clock();
// 		self_data.collidePairs.clear();
// 		Manager->collide(&self_data, CollisionFunction);
// 		stop = clock();
// 		cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
// 		sCP.clear();
// 		for(auto it = self_data.collidePairs.begin();it != self_data.collidePairs.end(); ++it)
// 		{
// 			std::string* label_1 = static_cast<std::string*>(it->first->getUserData());
// 			std::string* label_2 = static_cast<std::string*>(it->second->getUserData());
// 			sCP.emplace_back(make_pair(*label_1, *label_2));	
// 		}
// 		std::sort(sCP.begin(), sCP.end());
// 		this->selfCollisionPair.push_back(sCP);
// 		//////////////environment collision check///////////////////
// 		env_data.collidePairs.clear();
// 		for(int i = 0; i < this->envNum; ++i)
// 		{
// 			Manager->collide(this->environment[i], &env_data, CollisionFunction);
// 		}
// 		eCP.clear();
// 		for(auto it = env_data.collidePairs.begin();it != env_data.collidePairs.end(); ++it)
// 		{
// 			std::string* label_1 = static_cast<std::string*>(it->first->getUserData());
// 			std::string* label_2 = static_cast<std::string*>(it->second->getUserData());
// 			eCP.emplace_back(make_pair(*label_1, *label_2));
// 		}
// 		std::sort(eCP.begin(), eCP.end());
// 		this->envCollisionPair.push_back(eCP);
// 		start = clock();
// 		obj_data.collidePairs.clear();
// 		Manager->collide(this->workpiece, &obj_data, CollisionFunction);
// 		stop = clock();
// 		cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
// 		oCP.clear();
// 		for(auto it = obj_data.collidePairs.begin();it != obj_data.collidePairs.end(); ++it)
// 		{
// 			std::string* label_1 = static_cast<std::string*>(it->first->getUserData());
// 			std::string* label_2 = static_cast<std::string*>(it->second->getUserData());
// 			// std::cout<<"("<<*label_1<<", "<<*label_2<<")\n";
// 			oCP.emplace_back(make_pair(*label_1, *label_2));
// 		}
// 		std::sort(oCP.begin(), oCP.end());
// 		this->objCollisionPair.push_back(oCP);
// 		// printf("===========================================================\n");

// 	}
	
// }

bool RobotCollision::collisionQuery(const std::vector<fcl::Transform3<double>>& TransGroup)
{
	this->selfCollisionPair.clear();
	this->envCollisionPair.clear();
	this->objCollisionPair.clear();
	std::vector<std::pair<std::string, std::string>> sCP, eCP, oCP;
	// int link_data = TransGroup.size();
	// if (link_data != this->linkNum)
	// {
	// 	std::cerr<<"the data are inconsistant with robot link number\n";
	// 	return true;
	// }

	time_t start, stop;
	// printf("loading transforms......");

	start = clock();
	for (int i = 0; i < this->robotLinks.size(); ++i)
	{
		this->robotLinks[i]->setTransform(TransGroup[i]);
		this->robotLinks[i]->computeAABB();
	}
	this->Manager->update();
	stop = clock();
	double cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
	/////////////self collision check////////////////
	struct CollisionData self_data, env_data, obj_data;
	start = clock();
	self_data.collidePairs.clear();
	Manager->collide(&self_data, CollisionFunction);
	stop = clock();
	cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
	sCP.clear();
	for(auto it = self_data.collidePairs.begin();it != self_data.collidePairs.end(); ++it)
	{
		std::string* label_1 = static_cast<std::string*>(it->first->getUserData());
		std::string* label_2 = static_cast<std::string*>(it->second->getUserData());
		sCP.emplace_back(make_pair(*label_1, *label_2));	
	}
	std::sort(sCP.begin(), sCP.end());
	this->selfCollisionPair.push_back(sCP);
	//////////////environment collision check///////////////////
	env_data.collidePairs.clear();
	for(int i = 0; i < this->envNum; ++i)
	{
		Manager->collide(this->environment[i], &env_data, CollisionFunction);	
	}
	eCP.clear();
	for(auto it = env_data.collidePairs.begin();it != env_data.collidePairs.end(); ++it)
	{
		std::string* label_1 = static_cast<std::string*>(it->first->getUserData());
		std::string* label_2 = static_cast<std::string*>(it->second->getUserData());
		eCP.emplace_back(make_pair(*label_1, *label_2));
	}
	std::sort(eCP.begin(), eCP.end());
	this->envCollisionPair.push_back(eCP);
	//////////////object collision check///////////////////////
	start = clock();
	obj_data.collidePairs.clear();
	Manager->collide(this->workpiece, &obj_data, CollisionFunction);
	stop = clock();
	cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
	oCP.clear();
	for(auto it = obj_data.collidePairs.begin();it != obj_data.collidePairs.end(); ++it)
	{
		std::string* label_1 = static_cast<std::string*>(it->first->getUserData());
		std::string* label_2 = static_cast<std::string*>(it->second->getUserData());
		oCP.emplace_back(make_pair(*label_1, *label_2));
	}
	std::sort(oCP.begin(), oCP.end());
	this->objCollisionPair.push_back(oCP);
	// return oCP.size() + sCP.size() + eCP.size();
	return oCP.size();
}

bool RobotCollision::selfCollisionQuery(const std::vector<fcl::Transform3<double>>& TransGroup)
{
	// std::cerr<<"RC self coll 1\n";
	this->selfCollisionPair.clear();
	std::vector<std::pair<std::string, std::string>> sCP;
	// int link_data = TransGroup.size();
	// if (link_data != this->linkNum)
	// {
	// 	std::cerr<<"the data are inconsistant with robot link number\n";
	// 	return true;
	// }
	// std::cerr<<"RC self coll 2\n";	
	for (int i = 0; i < this->robotLinks.size(); ++i)
	{
		this->robotLinks[i]->setTransform(TransGroup[i]);
		// this->robotLinks[i]->computeAABB();
	}
	// std::cerr<<"RC self coll 3\n";	
	// this->Manager->update();
	struct CollisionData self_data;
	self_data.collidePairs.clear();
	// Manager->collide(&self_data, CollisionFunction);
	for(int it1 = 0; it1 < this->robotLinks.size(); ++it1)
	{
		for(int it2 = 0; it2 < this->robotLinks.size(); ++it2)
		{
			if (abs(it1 - it2) <= 1)
				continue;
			else
				CollisionFunction(this->robotLinks[it1], this->robotLinks[it2], &self_data);
		}
	}
// std::cerr<<"RC self coll 4\n";	
// std::cerr<<self_data.collidePairs.size()<<"\n";	
	sCP.clear();

	for(auto it = self_data.collidePairs.begin();it != self_data.collidePairs.end(); ++it)
	{
		// std::cerr<<"RC self coll 4.1\n";	
		std::string* label_1 = static_cast<std::string*>(it->first->getUserData());
		
		// std::cerr<< it->first->getUserData() <<"\n";	

		// std::cerr<<*label_1<<"\n";	

		std::string* label_2 = static_cast<std::string*>(it->second->getUserData());
		
		// std::cerr<<*label_2<<"\n";

		// std::cerr<<"RC self coll 4.2\n";
		
		// std::pair<std::string, std::string> dummy_pair = make_pair(*label_1, *label_2);
		
		// std::cerr<<"RC self coll 4.3\n";

		sCP.emplace_back(make_pair(*label_1, *label_2));
		// sCP.emplace_back(make_pair(*label_1, *label_2));	
		
		// std::cerr<<"RC self coll 4.4\n";
	}
// std::cerr<<"RC self coll 5\n";	
	std::sort(sCP.begin(), sCP.end());
	this->selfCollisionPair.push_back(sCP);
// std::cerr<<"RC self coll 6\n";	
// std::cerr<<sCP.size();
// std::cerr<<"RC self coll 7\n";	
	return sCP.size();
}

bool RobotCollision::envCollisionQuery(const std::vector<fcl::Transform3<double>>& TransGroup)
{	

	if (this->envNum == 0)
	{
		// printf("no environment data\n");
		return false;
	}

	for (auto i = 0; i < this->environment.size(); ++i)
	{
		this->environment[i]->setUserData(&this->envLabel[i]);
	}
	this->envCollisionPair.clear();
	std::vector<std::pair<std::string, std::string>> eCP;

	// int link_data = TransGroup.size();
	// if (TransGroup.size() != this->linkNum)
	// {
	// 	std::cerr<<"the data are inconsistant with robot link number\n";
	// 	std::cerr << TransGroup.size() << "\n";
	// 	// std::cerr << this->linkNum << "\n";
	// 	std::cerr << this->robotLinks.size() << "\n";
	// 	// return true;
	// }

	for (int i = 0; i < this->robotLinks.size(); ++i)
	{
		this->robotLinks[i]->setTransform(TransGroup[i]);
		// this->robotLinks[i]->computeAABB();
	}
	this->Manager->update();
	struct CollisionData env_data;
	env_data.collidePairs.clear();

	for(int i = 0; i < this->envNum; ++i)
	{
		Manager->collide(this->environment[i], &env_data, CollisionFunction);	

	}
	eCP.clear();

	for(auto it = env_data.collidePairs.begin();it != env_data.collidePairs.end(); ++it)
	{
	    // std::cout << "env -- ::5:: " << std::endl;				
		std::string* label_1 = static_cast<std::string*>(it->first->getUserData());
		std::string* label_2 = static_cast<std::string*>(it->second->getUserData());
		eCP.emplace_back(make_pair(*label_1, *label_2));
	}
	// std::cout << "env -- 6" << std::endl;	
	std::sort(eCP.begin(), eCP.end());
	this->envCollisionPair.push_back(eCP);
// std::cout << "env -- 7" << std::endl;		
	return eCP.size(); 
}

bool RobotCollision::objCollisionQuery(const std::vector<fcl::Transform3<double>>& TransGroup)
{

	if (this->workpiece == NULL)
	{
		// printf("no environment data\n");
		return false;
	}

	this->objCollisionPair.clear();
	std::vector<std::pair<std::string, std::string>> oCP;
	// int link_data = TransGroup.size();
	// if (link_data != this->linkNum)
	// {
	// 	std::cerr<<"the data are inconsistant with robot link number\n";
	// 	return true;
	// }
	// std::cout << "link size: " << this->robotLinks.size() << std::endl;
	for (int i = 0; i < this->robotLinks.size(); ++i)
	{
		this->robotLinks[i]->setTransform(TransGroup[i]);
		// if (i == this->robotLinks.size()-1){
		// if (i == 7){			
		// 	for(uint r = 0; r<4; ++r){
		// 		for(uint c = 0; c<4; ++c){
		// 			std::cout<<TransGroup[i](r,c)<<", ";			
		// 		}				
		// 		std::cout << std::endl;
		// 	}
		// 	printf("======================\n");

		// }
			// std::cout<<TransGroup[i](0,0)<<std::endl;
			// printf("[%.3f, %.3f, %.3f, %.3f\n
			// 	    %.3f, %.3f, %.3f, %.3f\n
			// 	    %.3f, %.3f, %.3f, %.3f\n
			// 	    %.3f, %.3f, %.3f, %.3f]\n", 
			// 	    TransGroup[i](0,0), TransGroup[i](0,1), TransGroup[i](0,2), TransGroup[i](0,3),
			// 	    TransGroup[i](1,0), TransGroup[i](1,1), TransGroup[i](1,2), TransGroup[i](1,3),
			// 	    TransGroup[i](2,0), TransGroup[i](2,1), TransGroup[i](2,2), TransGroup[i](2,3),
			// 	    TransGroup[i](3,0), TransGroup[i](3,1), TransGroup[i](3,2), TransGroup[i](3,3));
			
		// this->robotLinks[i]->computeAABB();
	}
	// this->Manager->update();
	struct CollisionData obj_data;
	obj_data.collidePairs.clear();
	// Manager->collide(this->workpiece, &obj_data, CollisionFunction);
	for(int it = 0; it < this->robotLinks.size(); ++it)
	{
		CollisionFunction(this->robotLinks[it], this->workpiece, &obj_data);
	}
	oCP.clear();
	for(auto it = obj_data.collidePairs.begin();it != obj_data.collidePairs.end(); ++it)
	{
		std::string* label_1 = static_cast<std::string*>(it->first->getUserData());
		std::string* label_2 = static_cast<std::string*>(it->second->getUserData());
		oCP.emplace_back(make_pair(*label_1, *label_2));
	}
	std::sort(oCP.begin(), oCP.end());
	this->objCollisionPair.push_back(oCP);
	return oCP.size();
}

void RobotCollision::distanceQuery(const std::vector<std::vector<fcl::Transform3<double>>>& TransGroup) 
{
	this->selfClosePair.clear();
	this->envClosePair.clear();
	this->objClosePair.clear();
	std::vector<std::pair<std::pair<std::string, std::string>,double>> sCP, eCP, oCP;
	// printf("checking transforms......\n");
	// int link_data = TransGroup.size();
	// if (link_data != this->linkNum)
	// {
	// 	std::cerr<<"the data are inconsistant with robot link number\n";
	// 	return;
	// }
	int frame_data = TransGroup[0].size();
	int frameNum = 0;
	for (int i = 0; i < this->robotLinks.size(); ++i)
	{
		if (TransGroup[i].size() != frame_data)
		{
			std::cerr<<"the frame number of link "<<i<<"is inconsistant with others\n";
			return;
		}
	}
	frameNum = frame_data;
	time_t start, stop;
	printf("loading transforms......\n");
	for(int j = 0; j < frameNum; ++j)
	{
		start = clock();
		for (int i = 0; i < this->robotLinks.size(); ++i)
		{
			this->robotLinks[i]->setTransform(TransGroup[i][j]);
			// this->robotLinks[i]->computeAABB(); //computing AABB will filter out some impossible candidates
												   //comment this out will show all distance pairs for self distance check
		}
		this->Manager->update();
		stop = clock();
		double cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
		struct DistanceData self_data, env_data, obj_data;
		////////////////self distance///////////////////////
		start = clock();
		self_data.closePairs.clear();
		Manager->distance(&self_data, DistanceFunction);
		stop = clock();
		cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
		sCP.clear();
		for(auto it = self_data.closePairs.begin();it != self_data.closePairs.end(); ++it)
		{
			std::string* label_1 = static_cast<std::string*>(it->first.first->getUserData());
			std::string* label_2 = static_cast<std::string*>(it->first.second->getUserData());
			double min_distance = it->second;
			sCP.emplace_back(make_pair(make_pair(*label_1, *label_2), min_distance));
		}
		std::sort(sCP.begin(), sCP.end());
		this->selfClosePair.push_back(sCP);
		////////////////////////environment distance/////////////////
		// start = clock();
		env_data.closePairs.clear();
		for(int i = 0; i < this->envNum; ++i)
		{
			Manager->distance(this->environment[i], &env_data, DistanceFunction);	
		}
		// Manager->distance(this->environment[0], &env_data, DistanceFunction);
		// stop = clock();
		// cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
		eCP.clear();
		for(auto it = env_data.closePairs.begin();it != env_data.closePairs.end(); ++it)
		{
			std::string* label_1 = static_cast<std::string*>(it->first.first->getUserData());
			std::string* label_2 = static_cast<std::string*>(it->first.second->getUserData());
			double min_distance = it->second;
			eCP.emplace_back(make_pair(make_pair(*label_1, *label_2), min_distance));
		}
		std::sort(eCP.begin(), eCP.end());
		this->envClosePair.push_back(eCP);
		///////////////////////object distance/////////////////////
		start = clock();
		obj_data.closePairs.clear(); 
		// Manager->distance(this->workpiece, &obj_data, DistanceFunction);//this funcion doesnt work well for some shapes
																		   //so is distancePatch introduced
		this->distancePatch(this->workpiece, &obj_data);
		stop = clock();
		cost_time = (double)(stop-start)/CLOCKS_PER_SEC;
		oCP.clear();
		for(auto it = obj_data.closePairs.begin();it != obj_data.closePairs.end(); ++it)
		{
			std::string* label_1 = static_cast<std::string*>(it->first.first->getUserData());
			std::string* label_2 = static_cast<std::string*>(it->first.second->getUserData());
			double min_distance = it->second;
			oCP.emplace_back(make_pair(make_pair(*label_1, *label_2), min_distance));
		}
		std::sort(oCP.begin(), oCP.end());
		this->objClosePair.push_back(oCP);
	}	
}

void RobotCollision::distancePatch(fcl::CollisionObject<double>* obj, void* cdata) const
{
	double dist = 0;
	for(int i = 0; i < this->robotLinks.size(); ++i)
	{
		DistanceFunction(this->workpiece, this->robotLinks[i], cdata, dist);
	}
}

void RobotCollision::selfDistanceQuery(const std::vector<fcl::Transform3<double>>& TransGroup, 
					   				   std::vector<std::pair<std::string, double>>& selfDistanceList)
{
	// int link_data = TransGroup.size();
	// if (link_data != this->linkNum)
	// {
	// 	std::cerr<<"the data are inconsistant with robot link number\n";
	// 	return;
	// }
	selfDistanceList.clear();
	for (int i = 0; i < this->robotLinks.size(); ++i)
	{
		this->robotLinks[i]->setTransform(TransGroup[i]);
	}
	// this->Manager->update();
	// struct DistanceData self_data;
	// self_data.closePairs.clear();
	// Manager->distance(&self_data, DistanceFunction);
	// sCP.clear();
	// for(auto it = self_data.closePairs.begin();it != self_data.closePairs.end(); ++it)
	// {
	// 	std::string* label_1 = static_cast<std::string*>(it->first.first->getUserData());
	// 	std::string* label_2 = static_cast<std::string*>(it->first.second->getUserData());
	// 	double min_distance = it->second;
	// 	sCP.emplace_back(make_pair(make_pair(*label_1, *label_2), min_distance));
	// }
	// std::sort(sCP.begin(), sCP.end());
	// for(int j = 0; j < sCP.size(); ++j)
	// {
	// 	printf("(%s, %s, %.3f)\n", sCP[j].first.first.c_str(),
	// 						 	   sCP[j].first.second.c_str(), 
	// 							   sCP[j].second);
	// }
	for(int it1 = 0; it1 < this->robotLinks.size(); ++it1)
	{
		struct DistanceOnePair self_data;
		self_data.min_distance = 1E9;
		for(int it2 = 0; it2 < this->robotLinks.size(); ++it2)
		{
			if (abs(it1 - it2) <= 1)
				continue;
			else
				OnePairDistanceFunction(this->robotLinks[it1], this->robotLinks[it2], &self_data);
		}
		std::string* label = static_cast<std::string*>(self_data.ObjectDistancePair.first->getUserData());
		selfDistanceList.push_back(make_pair(*label, self_data.ObjectDistancePair.second));
	}
}

void RobotCollision::envDistanceQuery(const std::vector<fcl::Transform3<double>>& TransGroup,
				  					  std::vector<std::pair<std::string, double>>& envDistanceList)
{
	envDistanceList.clear();
	if (this->envNum == 0)
	{
		// printf("no environment data\n");
		return;
	}
	// int link_data = TransGroup.size();
	// if (link_data != this->linkNum)
	// {
	// 	std::cerr<<"the data are inconsistant with robot link number\n";
	// 	return;
	// }
	for (int i = 0; i < this->robotLinks.size(); ++i)
	{
		this->robotLinks[i]->setTransform(TransGroup[i]);
	}
	for(int it1 = 0; it1 < this->robotLinks.size(); ++it1)
	{
		struct DistanceOnePair env_data;
		env_data.min_distance = 1E9;
		for(int it2 = 0; it2 < this->envNum; ++it2)
		{
			OnePairDistanceFunction(this->robotLinks[it1], this->environment[it2], &env_data);
		}
		std::string* label = static_cast<std::string*>(env_data.ObjectDistancePair.first->getUserData());
		envDistanceList.push_back(make_pair(*label, env_data.ObjectDistancePair.second));
	}
}

void RobotCollision::objDistanceQUery(const std::vector<fcl::Transform3<double>>& TransGroup, 
									  std::vector<double>& objDistanceList)
{
	objDistanceList.clear();
	if (this->workpiece == NULL)
	{
		// printf("no environment data\n");
		return;
	}
	// int link_data = TransGroup.size();
	// if (link_data != this->linkNum)
	// {
	// 	std::cerr<<"the data are inconsistant with robot link number\n";
	// 	return;
	// }
	for (int i = 0; i < this->robotLinks.size(); ++i)
	{
		this->robotLinks[i]->setTransform(TransGroup[i]);
	}
	for(int it = 0; it < this->robotLinks.size(); ++it)
	{
		struct DistanceOnePair obj_data;
		obj_data.min_distance = 1E9;
		OnePairDistanceFunction(this->robotLinks[it], this->workpiece, &obj_data);
		objDistanceList.push_back(obj_data.ObjectDistancePair.second);
	}

}

void RobotCollision::updateTool(const char* filepath)
{
	// if (idx >= this->linkNum)
	// {
	// 	std::cerr<<"can not update a link at this index"<<std::endl;
	// 	return;
	// }

	if(this->toolFlag == 0){
		addTool(filepath);
		return;
	}

	int idx = this->robotLinks.size() - 1;

	std::vector<fcl::Vector3<double>> p;
	std::vector<fcl::Triangle> t;	
	ReadMesh::loadModel(filepath, p, t);
	if(p.size() == 0 || t.size() == 0)
	{
		std::cerr<<"can not read the file!"<<std::endl;
		return;
	}
	std::shared_ptr<Model> geom = std::make_shared<Model>();
	geom->beginModel();
	geom->addSubModel(p, t);
	geom->endModel();
	
	this->robotLinks[idx] = new fcl::CollisionObject<double>(geom);
	this->robotLinks[idx]->setUserData(&this->toolLabel[0]);
	printf("update the tool (link %i) successfully,", idx);
	
	
	printf("this new tool has %li points and %li triangles\n", p.size(), t.size());
	printf("=========================================================\n");
	return;
}

void RobotCollision::addTool(const char* filepath)
{
	// if (idx >= this->linkNum)
	// {
	// 	std::cerr<<"can not update a link at this index"<<std::endl;
	// 	return;
	// }
	// this-; 
	// int idx = this->linkNum - 1;
	if (this->toolFlag == 1) 
		return;
	this->toolFlag = 1; 
	std::vector<fcl::Vector3<double>> p;
	std::vector<fcl::Triangle> t;	
	ReadMesh::loadModel(filepath, p, t);
	if(p.size() == 0 || t.size() == 0)
	{
		std::cerr<<"can not read the file!"<<std::endl;
		return;
	}
	std::shared_ptr<Model> geom = std::make_shared<Model>();
	geom->beginModel();
	geom->addSubModel(p, t);
	geom->endModel();

	fcl::CollisionObject<double>* tool = new fcl::CollisionObject<double>(geom);
	this->robotLinks.push_back(tool);
	this->robotLinks[this->robotLinks.size()-1]->setUserData(&this->toolLabel[0]);
	printf("add the tool successfully,");

	printf("this new tool has %li points and %li triangles\n", p.size(), t.size());
	printf("=========================================================\n");
	return;
}

void RobotCollision::updateToolPosition(const fcl::Transform3<double>& toolTransform)
{
	if (this->workpiece == NULL)
		return;
	else
		this->workpiece->setTransform(toolTransform);
}

void RobotCollision::removeTool()
{
	if (linkNum == 0||toolFlag == 0)
		return;
	this->robotLinks.pop_back();
	printf("remove the tool successfully\n");
	printf("=========================================================\n");
}

// void RobotCollision::updateEnvironment(int idx, const char* filepath)
// {
// 	if (idx >= this->envNum)
// 	{
// 		std::cerr<<"can not update an environment at this index"<<std::endl;
// 		return;
// 	}
// 	std::vector<fcl::Vector3<double>> p;
// 	std::vector<fcl::Triangle> t;
// 	ReadMesh::loadModel(filepath, p, t);
// 	if(p.size() == 0 || t.size() == 0)
// 	{
// 		std::cerr<<"can not read the file!"<<std::endl;
// 		return;
// 	}
// 	std::shared_ptr<Model> geom = std::make_shared<Model>();
// 	geom->beginModel();
// 	geom->addSubModel(p, t);
// 	geom->endModel();
// 	this->environment[idx] = new fcl::CollisionObject<double>(geom);
// 	this->environment[idx]->setUserData(&this->envLabel[idx]);
// 	printf("update environment %i successfully,", idx);
// 	printf("the new environment has %li points and %li triangles\n", p.size(), t.size());
// 	printf("=========================================================\n");
// 	return;	
// }

// void RobotCollision::updateEnvironment(int idx, const char* filepath, const fcl::Transform3<double> WP_transform)
// {
// 	if (idx >= this->envNum)
// 	{
// 		std::cerr<<"can not update an environment at this index"<<std::endl;
// 		return;
// 	}
// 	std::vector<fcl::Vector3<double>> p;
// 	std::vector<fcl::Triangle> t;
// 	ReadMesh::loadModel(filepath, p, t);
// 	if(p.size() == 0 || t.size() == 0)
// 	{
// 		std::cerr<<"can not read the file!"<<std::endl;
// 		return;
// 	}
// 	std::shared_ptr<Model> geom = std::make_shared<Model>();
// 	geom->beginModel();
// 	geom->addSubModel(p, t);
// 	geom->endModel();
// 	this->environment[idx] = new fcl::CollisionObject<double>(geom, WP_transform);
// 	this->environment[idx]->setUserData(&this->envLabel[idx]);
// 	printf("update environment %i successfully,", idx);
// 	printf("the new environment has %li points and %li triangles\n", p.size(), t.size());
// 	printf("=========================================================\n");
// 	return;		
// }

void RobotCollision::updateEnvironmentPosition(int idx, const fcl::Transform3<double>& envTransform)
{
	if ((idx < 0) || (idx >= this->envNum))
	{
		std::cerr<<"cannot update the environment position at this index\n";
		return;
	}
	else
	{
		this->environment[idx]->setTransform(envTransform);
		return;
	}
}
void RobotCollision::updateEnvironmentPosition(const std::vector<fcl::Transform3<double>>& envTransformGroup)
{
	if (this->envNum != envTransformGroup.size())
	{
		std::cerr<<"this number of transforms is inconsistant with the number of environment\n";
		return;
	}
	else
	{
		for (int i = 0; i < this->envNum; ++i)
		{
			this->environment[i]->setTransform(envTransformGroup[i]);
		}
		return;
	}
}

// void RobotCollision::removeEnvironment(int idx)
// {
// 	if (idx >= this->envNum)
// 	{
// 		std::cerr<<"can not remove an environment at this index\n"<<std::endl;
// 		return;
// 	}
// 	this->environment.erase(this->environment.begin()+(idx));
// 	this->envLabel.pop_back();
// 	for(int i = idx; i < this->environment.size(); ++i)
// 	{
// 		this->environment[i]->setUserData(&this->envLabel[i]);
// 	}
// 	this->envNum = this->environment.size();
// 	printf("remove environment %i successfully\n", idx);
// 	printf("=========================================================\n");
// 	return;
// }

// void RobotCollision::addEnvironment(const char* filepath)
// {
// 	std::vector<fcl::Vector3<double>> p;
// 	std::vector<fcl::Triangle> t;	
// 	ReadMesh::loadModel(filepath, p, t);
// 	if(p.size() == 0 || t.size() == 0)
// 	{
// 		std::cerr<<"can not read the file!"<<std::endl;
// 		return;
// 	}
// 	std::shared_ptr<Model> geom = std::make_shared<Model>();
// 	geom->beginModel();
// 	geom->addSubModel(p, t);
// 	geom->endModel();
// 	fcl::CollisionObject<double>* env = new fcl::CollisionObject<double>(geom);
// 	this->environment.push_back(env);
// 	this->envNum ++;
// 	int now_idx = this->envNum-1;
// 	std::ostringstream oss;
// 	oss<<"Environment"<<now_idx;
// 	this->envLabel.push_back(oss.str());
// 	this->environment[now_idx]->setUserData(&this->envLabel[now_idx]);
// }

void RobotCollision::addEnvironment(const char* filepath, const fcl::Transform3<double> WP_transform)
{

	std::vector<fcl::Vector3<double>> p;
	std::vector<fcl::Triangle> t;	
	ReadMesh::loadModel(filepath, p, t);
	if(p.size() == 0 || t.size() == 0)
	{
		std::cerr<<"can not read the file!"<<std::endl;
		return;
	}
	std::shared_ptr<Model> geom = std::make_shared<Model>();
	geom->beginModel();
	geom->addSubModel(p, t);
	geom->endModel();
	fcl::CollisionObject<double>* env = new fcl::CollisionObject<double>(geom, WP_transform);
	this->environment.push_back(env);
	this->envNum = this->environment.size();
	int now_idx = this->envNum - 1;
	std::ostringstream oss;
	oss<<"Environment"<<now_idx;
	this->envLabel.push_back(oss.str());
	// this->environment[now_idx]->setUserData(&this->envLabel[now_idx]);
	// this->environment[now_idx]->setUserData(&this->envLabel[]);

};


void RobotCollision::robotCollision(const RobotCollision* otherRobot, const std::vector<fcl::Transform3<double>>& TransGroup)
{

}

}

