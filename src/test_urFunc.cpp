#include <iostream>
#include <robot_utilities/ikfast_gateway.hpp>
#include <Eigen/Eigen>
#include <random>


int main(){
    Eigen::MatrixXd joint_config(6,1);
    joint_config << 0,0,0,0,0,0;
    Eigen::Matrix4d fk = ik_analytical::compute_FK(joint_config);
    std::cout<< "FK: \n" << fk << "\n";

    Eigen::VectorXd target(12);
    target.segment(0,3) = fk.block(0,3,3,1);
    target.segment(3,3) = fk.block(0,0,3,1);
    target.segment(6,3) = fk.block(0,1,3,1);
    target.segment(9,3) = fk.block(0,2,3,1);

    Eigen::MatrixXd solutions;
    bool status = false;
    ik_analytical::compute_IK(target,status,solutions);
    if(status)
        std::cout<< "IK Solved: Solutions are: \n" << solutions << "\n";
    return 0;
}