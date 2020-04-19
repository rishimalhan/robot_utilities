// Point-To-Point Trajectory Planner for Serial Link Manipulator
// author: pradeepr@usc.edu, akabir@usc.edu

#ifndef STLtoPCL_H
#define STLtoPCL_H

#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <ctime>
#include <igl/readSTL.h>
#include <igl/readOBJ.h>
#include "stdlib.h"
#include <cstdlib>


namespace STLtoPCL{

Eigen::MatrixXd gen_PCL_from_STL(std::string stl_file_name, int xgap, int ygap); // nx6 
Eigen::MatrixXd gen_positions(Eigen::MatrixXd in_pts); // nx3 
Eigen::MatrixXd generate_grid_points(double pathgap_x, double pathgap_y, double xmin, double ymin, double xmax, double ymax);
Eigen::MatrixXd generate_pointcloud(Eigen::MatrixXd v, Eigen::MatrixXd f, Eigen::MatrixXd n, double gap_x, double gap_y);
Eigen::MatrixXd generate_pointcloud(Eigen::MatrixXd v, Eigen::MatrixXd f, double gap_x, double gap_y);
Eigen::MatrixXd add_pts(Eigen::MatrixXd tri, Eigen::MatrixXd grid_pts);
Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::MatrixXd T_mat);
Eigen::Matrix3d rot_y(double t);
Eigen::Matrix3d rot_x(double t);
Eigen::MatrixXd InPoly(Eigen::MatrixXd q, Eigen::MatrixXd p);
bool lines_intersect(double l1[2][2], double l2[2][2]);
std::vector<int> find_idx(Eigen::VectorXi vec);
std::vector<int> find_idx(Eigen::MatrixXd vec);
Eigen::VectorXd linsp(double strt, double end, double stp);
void file_write(std::string file_name, const Eigen::MatrixXd& mat);

}
#endif 