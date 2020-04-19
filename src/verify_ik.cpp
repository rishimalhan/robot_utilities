/* AUTHORED BY: Rishi Malhan
Center for Advanced Manufacturing
University of Southern California, LA, USA.
EMAIL: rmalhan@usc.edu */


#include <iostream>
#include <robot_utilities/ikfast_gateway.hpp>
#include <Eigen/Eigen>
#include <random>


int main(int argc, char** argv)
{
    int dof = 6;


    std::random_device rd; // obtain a random number from hardware
    std::mt19937 eng(rd()); // seed the generator
    std::uniform_real_distribution<> distr(-M_PI, M_PI); // define the range
    Eigen::MatrixXd joint_config;

    if (dof==7)
        joint_config.resize(7,1);
    else
        joint_config.resize(6,1);
    Eigen::Matrix4d tf;
    Eigen::Matrix4d ik_tf;
    bool isValid; // Success flag. True is solution found
    
    bool status = true;
    if (dof==7)
        joint_config << 0,0,0,0,0,0,0;
    else
        joint_config << 0,0,0,0,0,0;
    std::cout<< "FK at zero location of all joints:  " << std::endl;
    std::cout<< ik_analytical::compute_FK(joint_config)  << std::endl;
    Eigen::VectorXd target(12);

    for (int i=0; i<100000; ++i)
    {
        if (dof==7)
            joint_config.col(0) << distr(eng),distr(eng),distr(eng),distr(eng),distr(eng),distr(eng),distr(eng); // 7DOF
        else
            joint_config.col(0) << distr(eng),distr(eng),distr(eng),distr(eng),distr(eng),distr(eng); // 6DOF
        tf = ik_analytical::compute_FK(joint_config);   
        target.segment(0,3) = tf.block(0,3,3,1);
        target.segment(3,3) = tf.block(0,0,3,1);
        target.segment(6,3) = tf.block(0,1,3,1);
        target.segment(9,3) = tf.block(0,2,3,1); 
        Eigen::MatrixXd sol_mat; // N solutions. Nxdof matrix
        ik_analytical::compute_IK(target, isValid, sol_mat);
        if (isValid)
        {
            for (int j=0; j<sol_mat.rows();++j)
            {
                joint_config.col(0) = sol_mat.row(j).transpose();
                ik_tf = ik_analytical::compute_FK(joint_config);
                // True if equal
                if (!ik_tf.isApprox(tf,1e-3))
                {
                    std::cout<< "Error in files. Invalid FK for IK solution" << std::endl;
                    status = false;

                    std::cout<< "Reference FK: " << std::endl;
                    std::cout<< tf << std::endl;
                    std::cout<< std::endl;

                    std::cout<< "Ik solutions: \n" << sol_mat << std::endl;
                    std::cout<< std::endl;

                    std::cout<< "FK from Ik solution: " << std::endl;
                    std::cout<< ik_tf << std::endl;
                    break;
                }           
            }
        }
        else
        {
            std::cout<< "Error in files. IK doesnot exist for FK" << std::endl;
            status = false;
            break;
        }

        if (!status)
            break;
    }

    if (status)
        std::cout<< "Verification Successful" << std::endl;
    return 0;
}