// author: akabir@usc.edu

#ifndef SerialLink_Manipulator_H
#define SerialLink_Manipulator_H

#include <iostream>
#include <iomanip>
#include <vector>

#include <fstream>
#include <array>
#include <valarray>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/QR>

#include <boost/date_time.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/foreach.hpp>

#include <typeinfo>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

// #include <fcl/shape/geometric_shapes.h>
// #include <fcl/shape/geometric_shapes_utility.h>
// #include <fcl/BVH/BVH_model.h>
// #include <fcl/collision.h>
// #include <fcl/distance.h>

#include <ros/ros.h>
#include <urdf/model.h>
#include <limits>

#include <robot_utilities/Data_Format_Mapping.hpp>

namespace SerialLink_Manipulator{

// Jacobian Sequence is X, Y, Z, Rx, Ry, Rz

class SerialLink_Manipulator
{
    private:

    public:
        SerialLink_Manipulator();
        SerialLink_Manipulator(std::string urdf_path, KDL::Frame base_frame, KDL::Frame tcp_frame, std::string base_link, std::string tip_link);
        ~SerialLink_Manipulator();
        double KDL_IK_eps;
        std::string URDFpath;
        std::string RobotName;                
        std::string BaseLink;
        std::string TipLink; 
        std::string dummy_str; 

        int NrOfJoints;
        int NrOfLinks;

        urdf::Model RobotModel;
        std::vector<boost::shared_ptr<urdf::Link> > links;
        std::vector<std::string> LinkNames;
        std::vector<urdf::JointConstSharedPtr> joints;
        std::vector<std::string> JointNames;
        std::vector< std::vector< double> > link_pts;
        
        KDL::Frame BaseFrame;
        KDL::Frame TCPFrame;
        KDL::Tree RobotModelTree;
        KDL::Chain RobotChain;
        std::vector<KDL::Chain> RobotChainsAllLinks;        
        KDL::JntArray Joints_ll, Joints_ul, Joints_ntrl; //lower joint limits, upper joint limits
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> KDL_FKSolver;
        boost::scoped_ptr<KDL::ChainJntToJacSolver> KDL_JacSolver;
        boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> KDL_VIKSolver;
        boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> KDL_IKSolver;

        std::vector< boost::shared_ptr<KDL::ChainFkSolverPos_recursive> > KDL_FKSolver_all_links;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> KDL_FKSolverTEMP;
        std::vector< KDL::ChainFkSolverPos_recursive > KDL_FKSolver_all_links_obj;

        double fRand(double min, double max);
        void spawn_KDLRobotModel_from_urdf();
        void spawn_KDLTree_from_KDLRobotModel();
        void spawn_KDLChain_from_link0_to_all_links();
        void spawn_KDLRobotChain_from_link0_to_linkee();
        void populate_robot_joint_limits();
        void getRandJointArray(KDL::JntArray & randJoints);
        void spawn_KDL_FK_JAC_IK_solvers();
        KDL::JntArray convert_double_joints_to_KDL_joint(double* joints);
        bool is_valid_joint_angle(KDL::JntArray jointAngles);

        void FK_KDL_all_links (KDL::JntArray & joints, std::vector<KDL::Frame> & linkFrames);
        void FK_KDL_TCP (KDL::JntArray & joints, KDL::Frame & tcpFrame);
        void FK_KDL_Flange (KDL::JntArray & joints, KDL::Frame & eeFrame);
        void FK_Analytical ();
        void Jac_KDL (KDL::JntArray & joints, KDL::Jacobian & KDLJacobian);
        void IK_KDL (KDL::JntArray & seed_joints, KDL::Frame & eeFrame, KDL::JntArray & ikresult);

        Eigen::MatrixXd get_robot_FK(Eigen::MatrixXd q);
        std::vector<Eigen::MatrixXd> get_robot_FK_all_links(Eigen::MatrixXd q);
        Eigen::MatrixXd get_robot_Jacobian(Eigen::MatrixXd q);
        Eigen::MatrixXd get_robot_IK(Eigen::MatrixXd T_tcp);

        void print_KDL_JointArray(KDL::JntArray & q);
        void print_KDL_Frame(KDL::Frame & kdlFrame);
        void print_KDL_Jacobian(KDL::Jacobian & KDLjacobian);

        double get_collision_score(double * joints);

        void updateTCP(KDL::Frame tcp_frame);
};


}

#endif


