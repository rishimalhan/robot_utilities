// Point-To-Point Trajectory Planner for Serial Link Manipulator
// author: pradeepr@usc.edu, akabir@usc.edu

#ifndef DFMapping_H
#define DFMapping_H

#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace DFMapping{

    // joint angles
    std::vector<double> KDLJoints_to_STDvector(KDL::JntArray joints);
    KDL::JntArray STDvector_to_KDLJoints(std::vector<double> joints);
    Eigen::MatrixXd KDLJoints_to_Eigen(KDL::JntArray joints);
    KDL::JntArray Eigen_to_KDLJoints(Eigen::MatrixXd joints);
        
    // Coordinate Frames
    std::vector<double> KDLFrame_to_STDvector(KDL::Frame kdl_frame);
    KDL::Frame STDvector_to_KDLFrame(std::vector<double> frame);
    Eigen::MatrixXd KDLFrame_to_Eigen(KDL::Frame kdl_frame);
    KDL::Frame Eigen_to_KDLFrame(Eigen::MatrixXd frame);
    Eigen::MatrixXd KDLJacobian_to_Eigen(KDL::Jacobian jac);

    // General Matrix
    Eigen::MatrixXd STD_Vector_of_Vector_to_Eigen_Matrix(std::vector<std::vector<double>>);
    std::vector<std::vector<double>> Eigen_Matrix_to_STD_Vector_of_Vector(Eigen::MatrixXd);

}
#endif 