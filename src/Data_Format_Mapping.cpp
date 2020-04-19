// mapping between KDL Eigen Vector data formats for serial link manipulators

#include <robot_utilities/Data_Format_Mapping.hpp>

namespace DFMapping{

///////////////////////////////////////////////////////////

std::vector<double> KDLJoints_to_STDvector(KDL::JntArray joints){
    std::vector<double> joints_vec;
    joints_vec.clear();
    for (uint i=0; i < joints.rows(); ++i){
        joints_vec.push_back(joints(i));
    }
    return joints_vec;
};

///////////////////////////////////////////////////////////

KDL::JntArray STDvector_to_KDLJoints(std::vector<double> joints){
    KDL::JntArray joints_kdl = KDL::JntArray(joints.size());
    for (uint i=0; i < joints.size(); ++i){
        joints_kdl(i) = joints[i];
    }
    return joints_kdl;
};

///////////////////////////////////////////////////////////

Eigen::MatrixXd KDLJoints_to_Eigen(KDL::JntArray joints){
    Eigen::MatrixXd joints_eig(joints.rows(),1);
    for (uint i=0; i < joints.rows(); ++i){
        joints_eig(i,0) = joints(i);
    }
    return joints_eig;        
};

///////////////////////////////////////////////////////////

KDL::JntArray Eigen_to_KDLJoints(Eigen::MatrixXd joints){
    KDL::JntArray joints_kdl = KDL::JntArray(joints.rows());
    for (uint i=0; i < joints.rows(); ++i){
        joints_kdl(i) = joints(i,0);
    }
    return joints_kdl;        
};

///////////////////////////////////////////////////////////

std::vector<double> KDLFrame_to_STDvector(KDL::Frame frame){
    std::vector<double> frame_vec;
    frame_vec.clear();
    for (uint r = 0; r<3; ++r){        
        frame_vec.push_back(frame(r, 3));
    }
    for (uint c = 0; c<3; ++c){
        for (uint r = 0; r<3; ++r){
            frame_vec.push_back(frame(r, c));
        }
    }
    return frame_vec;
};

///////////////////////////////////////////////////////////

KDL::Frame STDvector_to_KDLFrame(std::vector<double> frame){

    KDL::Rotation kdlR = KDL::Rotation(frame[3],frame[4],frame[5],frame[6],frame[7],frame[8],frame[9],frame[10],frame[11]);
    KDL::Vector kdlt = KDL::Vector(frame[0], frame[1], frame[2]);
    KDL::Frame frame_kdl = KDL::Frame(kdlR, kdlt);
    return frame_kdl;
};

///////////////////////////////////////////////////////////

Eigen::MatrixXd KDLFrame_to_Eigen(KDL::Frame frame){
    Eigen::MatrixXd frame_eig(4,4);
    frame_eig.block(3,0,1,4) << 0,0,0,1;
    for (uint c = 0; c<4; ++c){
        for (uint r = 0; r<3; ++r){
            frame_eig(r, c) = frame(r, c);
        }
    }        
    return frame_eig;
};

///////////////////////////////////////////////////////////

KDL::Frame Eigen_to_KDLFrame(Eigen::MatrixXd frame){
    KDL::Rotation kdlR = KDL::Rotation(frame(0,0), frame(0,1), frame(0,2), frame(1,0), frame(1,1), frame(1,2), frame(2,0), frame(2,1), frame(2,2));
    KDL::Vector kdlt = KDL::Vector(frame(0,3), frame(1,3), frame(2,3));
    KDL::Frame frame_kdl = KDL::Frame(kdlR, kdlt);
    return frame_kdl;
};

///////////////////////////////////////////////////////////

Eigen::MatrixXd KDLJacobian_to_Eigen(KDL::Jacobian jac){
    Eigen::MatrixXd jac_eig(jac.rows(),jac.columns());
    for (int i = 0; i < jac.rows(); ++i){
        for (int j = 0; j < jac.columns(); ++j) {
            jac_eig(i,j) = jac(i, j);
        }
    } 
    return jac_eig;
};

///////////////////////////////////////////////////////////

Eigen::MatrixXd STD_Vector_of_Vector_to_Eigen_Matrix(std::vector<std::vector<double>> vvd){
    // assuming outer vector contains rows, and inner vector contains columns
    Eigen::MatrixXd emd(vvd.size(), vvd[0].size());
    for (uint r=0;r<vvd.size();++r){
        for (uint c=0;c<vvd[0].size();++c){
            emd(r,c) = vvd[r][c];
        }
    }
    return emd;
};

///////////////////////////////////////////////////////////

std::vector<std::vector<double>> Eigen_Matrix_to_STD_Vector_of_Vector(Eigen::MatrixXd emd){
    std::vector<std::vector<double>> vvd; 
    vvd.clear();
    std::vector<double> vvd_row; 
    for (uint r=0;r<emd.rows();++r){
        vvd_row.clear();
        for(uint c=0; c<emd.cols();++c){
            vvd_row.push_back(emd(r,c));
        }
        vvd.push_back(vvd_row);
    }
    return vvd;
};

///////////////////////////////////////////////////////////

}