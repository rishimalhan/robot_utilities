#ifndef ROBOTCOLLISION_H
#define ROBOTCOLLISION_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <sys/types.h>
#include <dirent.h>
#include <fcl/fcl.h>
#include <math.h>

#include <robot_utilities/ReadMesh.hpp>

namespace RCn{

class RobotCollision
{
public:
	RobotCollision(void);
	virtual ~RobotCollision(void);
	std::vector<fcl::CollisionObject<double>*> robotLinks, environment;
	fcl::CollisionObject<double>* workpiece;
	fcl::BroadPhaseCollisionManager<double>* Manager;
	int linkNum = 0; 
	int envNum = 0;
	int toolFlag = 0;
	//tag
	std::vector<std::string> linkLabel, envLabel;
	std::vector<std::string> toolLabel = {"Tool"};
	std::string objectLabel[1] = {"Object"};
	std::vector<std::vector<std::pair<std::string, std::string>>> selfCollisionPair, envCollisionPair, objCollisionPair;
	std::vector<std::vector<std::pair<std::pair<std::string, std::string>,double>>> selfClosePair, envClosePair, objClosePair;
private:
	void distancePatch(fcl::CollisionObject<double>* obj, void* cdata) const;
public:
	void loadRobot(const char* filepath);
	void loadWorkpiece(const char* filepath);
	void loadWorkpiece(const char* filepath, const fcl::Transform3<double> WP_transform);
	void init_basic();
	void init(const char* robotPath, const char* workpiecePath);
	void init(const char* robotPath, const char* workpiecePath, const fcl::Transform3<double> WP_transform);
	//////////collision query///////
	// void collisionQuery(const std::vector<std::vector<fcl::Transform3<double>>>& TransGroup);
	bool collisionQuery(const std::vector<fcl::Transform3<double>>& TransGroup);
	bool selfCollisionQuery(const std::vector<fcl::Transform3<double>>& TransGroup);
	bool envCollisionQuery(const std::vector<fcl::Transform3<double>>& TransGroup);
	bool objCollisionQuery(const std::vector<fcl::Transform3<double>>& TransGroup);	
	//////////distance query////////
	void distanceQuery(const std::vector<std::vector<fcl::Transform3<double>>>& TransGroup); 
	void selfDistanceQuery(const std::vector<fcl::Transform3<double>>& TransGroup, std::vector<std::pair<std::string, double>>& selfDistanceList);
	void envDistanceQuery(const std::vector<fcl::Transform3<double>>& TransGroup, std::vector<std::pair<std::string, double>>& envDistanceList);
	void objDistanceQUery(const std::vector<fcl::Transform3<double>>& TransGroup, std::vector<double>& objDistanceList);
	/////////update data///////////
	void addTool(const char* filepath);
	void updateTool(const char* filepath);
	void updateToolPosition(const fcl::Transform3<double>& toolTransform);
	void removeTool();
	// void updateEnvironment(int idx, const char* filepath);
	// void updateEnvironment(int idx, const char* filepath, const fcl::Transform3<double> WP_transform);
	void updateEnvironmentPosition(int idx, const fcl::Transform3<double>& envTransform);
	void updateEnvironmentPosition(const std::vector<fcl::Transform3<double>>& envTransformGroup);	
	// void removeEnvironment(int idx);
	void addEnvironment(const char* filepath);
	void addEnvironment(const char* filepath, const fcl::Transform3<double> WP_transform);
	////////robot collision///////
	void robotCollision(const RobotCollision* otherRobot, const std::vector<fcl::Transform3<double>>& TransGroup);
};

}
#endif //ROBOTCOLLISION_H