#ifndef READMESH_H
#define READMESH_H
#include <iostream>
#include <Eigen/Eigen>
#include <fcl/fcl.h>
#include <igl/readSTL.h>
#include <igl/readOBJ.h>

namespace ReadMesh
{
	void loadSTLFile(const char* filename, std::vector<fcl::Vector3<double>>& points, std::vector<fcl::Triangle>& triangles);
	void loadOBJFile(const char* filename, std::vector<fcl::Vector3<double>>& points, std::vector<fcl::Triangle>& triangles);
	void loadModel(const char* filename, std::vector<fcl::Vector3<double>>& points, std::vector<fcl::Triangle>& triangles);
}

#endif //READMESH_H