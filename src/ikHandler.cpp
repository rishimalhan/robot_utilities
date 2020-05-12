///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUTHOR: RISHI MALHAN
// CENTER FOR ADVANCED MANUFACTURING
// UNIVERSITY OF SOUTHERN CALIFORNIA
// EMAIL: rmalhan0112@gmail.com
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#include <robot_utilities/ikHandler.hpp>
#include <limits>

// Definitions
// defualt nlopt fuctions begin
// Error function only minimizes the obejective. So for max use negative.
double err_func_gateway(const std::vector<double>& x, std::vector<double>& grad, void* data) 
{
    // Because we wanted a Class
    // without static members, but NLOpt library does not support
    // passing methods of Classes, we use these auxilary functions.
    ikHandler *gateway = (ikHandler *) data;
    return gateway->obj_func(x,grad);
}
// defualt nlopt fuctions end


ikHandler::ikHandler(SerialLink_Manipulator::SerialLink_Manipulator* _robot){
    robot = _robot;
    OptVarDim = robot->NrOfJoints;
    jt_ll.resize(OptVarDim);
    jt_ul.resize(OptVarDim);
    init_guess.resize(OptVarDim);
    // Default initial guess
    for (int i=0; i<OptVarDim; ++i){
        init_guess(i) = 0;
        jt_ll(i) = robot->Joints_ll(i);
        jt_ul(i) = robot->Joints_ul(i);
    }
    closest_sol.resize(OptVarDim);
    //choose optimizer
    // alg_type = nlopt::LN_NEWUOA;
    // alg_type = nlopt::LN_NEWUOA_BOUND;
    // alg_type = nlopt::LD_MMA;
    // alg_type = nlopt::LD_TNEWTON_PRECOND_RESTART;
    // alg_type = nlopt::LN_BOBYQA;
    // alg_type = nlopt::LN_COBYLA;
    alg_type = nlopt::LD_SLSQP;
    // alg_type = nlopt::LD_LBFGS;
    // alg_type = nlopt::GN_ISRES;
    optXtolRel = 1e-12;
    gradH = 1e-8;
    // Lower and Upper Bounds for joints
    OptVarlb.resize(OptVarDim);
    OptVarub.resize(OptVarDim);
    for (int i=0; i<OptVarDim; ++i){
        OptVarlb[i] = jt_ll(i);
        OptVarub[i] = jt_ul(i);
    }
    // Determine dimension of all the constraints. If constraints are applied successively,
    // Set this to max dimension and keep flag in constr_func which will keep 
    // non-enforced constraints zero.
    optimizer = nlopt::opt(alg_type, OptVarDim);
    optimizer.set_xtol_rel(optXtolRel);
    optimizer.set_min_objective(err_func_gateway, this);
    optimizer.set_lower_bounds(OptVarlb);
    optimizer.set_upper_bounds(OptVarub);
    optimizer.set_ftol_rel(1e-12);   // Tolerance in objective function change.
    optimizer.set_maxeval(2000);
    status = false;
    target.resize(12);
    // mapped_target.resize(12);

    // Inverse of world_T_robot_base
    world_T_robBase = Eigen::MatrixXd::Identity(4,4);
    world_T_robBase = DFMapping::KDLFrame_to_Eigen(robot->BaseFrame);
    world_T_robBase.block(0,0,3,1) /= world_T_robBase.block(0,0,3,1).norm();
    world_T_robBase.block(0,1,3,1) /= world_T_robBase.block(0,1,3,1).norm();
    world_T_robBase.block(0,2,3,1) /= world_T_robBase.block(0,2,3,1).norm();

    // std::cout<< "Robot base frame is at: \n" << world_T_robBase << "\n";
    robBase_T_world = Eigen::MatrixXd::Identity(4,4);
    robBase_T_world.block(0,0,3,3)  = world_T_robBase.block(0,0,3,3).transpose();
    robBase_T_world.block(0,3,3,1) = -world_T_robBase.block(0,0,3,3).transpose()*world_T_robBase.block(0,3,3,1);

    robBase_T_world.block(0,0,3,1) /= robBase_T_world.block(0,0,3,1).norm();
    robBase_T_world.block(0,1,3,1) /= robBase_T_world.block(0,1,3,1).norm();
    robBase_T_world.block(0,2,3,1) /= robBase_T_world.block(0,2,3,1).norm();

    std::cout<< "#########################################################################\nikHandler initialization complete\n" <<
    "#########################################################################\n";

    urIKPatch = false;
    if (OptVarDim<7)
        genPerm(OptVarDim);
    // Eigen::MatrixXd jt_config(6,1);
    // jt_config<< 0,-90,0,0,0,0;
    // jt_config *= (M_PI/180);
    // std::cout<< "KDL FK: \n";
    // KDL::Frame frame;
    // KDL::JntArray fuck = DFMapping::Eigen_to_KDLJoints(jt_config);
    // robot->FK_KDL_Flange(fuck,frame);
    // std::cout<< DFMapping::KDLFrame_to_Eigen(frame) << "\n\n";
    // std::cout<< "IKfast FK: \n";
    // std::cout<< ik_analytical::compute_FK(jt_config) << "\n";
}

ikHandler::~ikHandler() {};

double ikHandler::obj_func(const std::vector<double>& x, std::vector<double>& grad)
{
    double err = err_func(x);
    if (!grad.empty()) {
        std::vector<double> xph = x;
        for (uint i=0; i < x.size(); ++i)
        {
            xph[i] += gradH;
            grad[i] = (err_func(xph)-err)/gradH;
            xph[i] -= gradH;
        }
    }    
    return err;
}

double ikHandler::err_func(const std::vector<double>& x)
{
    KDL::JntArray joint_config = DFMapping::STDvector_to_KDLJoints(x);
    // robot->FK_KDL_TCP(joint_config, FK_tcp);
    robot->FK_KDL_Flange(joint_config, FK_tcp);
    return  (pow(target(0)-FK_tcp(0,3),2) + pow(target(1)-FK_tcp(1,3),2) + pow(target(2)-FK_tcp(2,3),2) + // Translation Error
              pow( 1 - (target(3)*FK_tcp(0,0) + target(4)*FK_tcp(1,0) + target(5)*FK_tcp(2,0)) , 2 ) + //bx
               pow( 1 - (target(6)*FK_tcp(0,1) + target(7)*FK_tcp(1,1) + target(8)*FK_tcp(2,1)) , 2 ) +    //by
               pow( 1 - (target(9)*FK_tcp(0,2) + target(10)*FK_tcp(1,2) + target(11)*FK_tcp(2,2)) , 2 ) ); //bz
};

void ikHandler::setTcpFrame(const Eigen::MatrixXd& _tcpframe){
    robot->TCPFrame = DFMapping::Eigen_to_KDLFrame(_tcpframe);
};

Eigen::VectorXd ikHandler::getFFTarget(){
    return target;    
};


bool ikHandler::isPresent(Eigen::MatrixXi perm_mat, Eigen::MatrixXi row_vec){
    for (int i=0; i<perm_mat.rows(); ++i)
        if (perm_mat.row(i)==row_vec)
            return true;
    return false;
}
void ikHandler::genPerm(int dof){
    Eigen::MatrixXi perm_mat;
    perm_mat.conservativeResize(2,dof);
    for (int i=0; i<dof; ++i){
        perm_mat(0,i) = 0;
        perm_mat(1,i) = 1;
    }

    for (int i=1; i<perm_mat.rows(); ++i){
        for (int j=0; j<dof; ++j){
            Eigen::MatrixXi row_vec;

            row_vec = perm_mat.row(i);
            row_vec(0,j) *= 0;
            if (!isPresent(perm_mat,row_vec)){
                // Add this as a new permutation
                perm_mat.conservativeResize(perm_mat.rows()+1,dof);
                perm_mat.row(perm_mat.rows()-1) = row_vec;
            }

            row_vec = perm_mat.row(i);
            row_vec(0,j) *= 1;
            if (!isPresent(perm_mat,row_vec)){
                // Add this as a new permutation
                perm_mat.conservativeResize(perm_mat.rows()+1,dof);
                perm_mat.row(perm_mat.rows()-1) = row_vec;
            }

            row_vec = perm_mat.row(i);
            row_vec(0,j) *= -1;
            if (!isPresent(perm_mat,row_vec)){
                // Add this as a new permutation
                perm_mat.conservativeResize(perm_mat.rows()+1,dof);
                perm_mat.row(perm_mat.rows()-1) = row_vec;
            }
        }
    }
    permutations = 2*M_PI*perm_mat.cast<double>();
};
void ikHandler::enable_URikPatch(){urIKPatch = true;};
void ikHandler::disable_URikPatch(){urIKPatch = false;};
void ikHandler::apply_URikPatch(Eigen::MatrixXd &solutions){
    int perm_rows = permutations.rows();
    int sol_cols = solutions.cols();
    int sol_rols = solutions.rows();
    Eigen::MatrixXd new_sols(perm_rows*sol_rols, sol_cols);
    for (int i=0; i<solutions.rows(); ++i)
        new_sols.block(i*perm_rows,0,perm_rows,sol_cols) = solutions.row(i).replicate(perm_rows,1) + permutations;
    solutions = new_sols;
};






//////////////////////// MAIN FUNCTION //////////////////////////////////////////////////
bool ikHandler::solveIK(Eigen::VectorXd _target){
    // Find target with respect to robot base
    Eigen::Matrix4d target_robBase = Eigen::Matrix4d::Identity();
    target_robBase.block(0,0,3,1) = _target.segment(3,3);
    target_robBase.block(0,1,3,1) = _target.segment(6,3);
    target_robBase.block(0,2,3,1) = _target.segment(9,3);
    target_robBase.block(0,3,3,1) = _target.segment(0,3);
    target_robBase = robBase_T_world*target_robBase;


    // Solve IK for target wrt robot base frame
    // target is where the flange needs to be wrt robot base
    status = true;
    // Numerical IK
    if (OptVarDim > 6 || useNumIK){
        // Computing tcp_T_ff for using it to define a target for flange based on current TCP
        Eigen::MatrixXd TCPFrame = DFMapping::KDLFrame_to_Eigen(robot->TCPFrame); // We need to take it's inverse
        Eigen::Matrix4d tcp_T_ff = Eigen::Matrix4d::Identity();
        // Inverse
        tcp_T_ff.block(0,0,3,3)  = TCPFrame.block(0,0,3,3).transpose();
        tcp_T_ff.block(0,3,3,1) = -TCPFrame.block(0,0,3,3).transpose()*TCPFrame.block(0,3,3,1);
        // Inverse done
        tcp_T_ff.block(0,0,3,1) /= tcp_T_ff.block(0,0,3,1).norm();
        tcp_T_ff.block(0,1,3,1) /= tcp_T_ff.block(0,1,3,1).norm();
        tcp_T_ff.block(0,2,3,1) /= tcp_T_ff.block(0,2,3,1).norm();

        // Transform target to flange
        target_robBase = target_robBase*tcp_T_ff;

        target.segment(3,3) = target_robBase.block(0,0,3,1);
        target.segment(6,3) = target_robBase.block(0,1,3,1);
        target.segment(9,3) = target_robBase.block(0,2,3,1);
        target.segment(0,3) = target_robBase.block(0,3,3,1);

        // Make the Cartesian Axes Perpendicular. Evaluate // by = bz x bx and // bx = by x bz
        // by
        target(6) = (target(10)*target(5)) - (target(11)*target(4));
        target(7) = (target(11)*target(3)) - (target(9)*target(5));
        target(8) = (target(9)*target(4)) - (target(10)*target(3));
        // bx
        target(3) = (target(7)*target(11)) - (target(8)*target(10));
        target(4) = (target(8)*target(9)) - (target(6)*target(11));
        target(5) = (target(6)*target(10)) - (target(7)*target(9));
        target.segment(3,3) /= target.segment(3,3).norm(); //bx
        target.segment(9,3) /= target.segment(9,3).norm(); //by
        target.segment(6,3) /= target.segment(6,3).norm(); //bz

        solution.resize(OptVarDim,1);
        
        std::vector<double> iterator(OptVarDim);
        for (int i=0; i<OptVarDim; ++i)
            iterator[i] = init_guess(i);

        // NLopt routine
        bool optSuccess = false;
        try{
            
            nlopt::result result = optimizer.optimize(iterator, f_val);
            optSuccess = true;
        }
        catch(std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }

        if (!optSuccess || f_val > 1e-10)
            status = false;

        for (int i=0; i < OptVarDim; i++)
            solution(i,0) = iterator[i];
        closest_sol = solution.col(0);
        return status;
    }
    // Analytical IK
    else{
        // Computing tcp_T_ff for using it to define a target for flange based on current TCP
        Eigen::MatrixXd TCPFrame = DFMapping::KDLFrame_to_Eigen(robot->TCPFrame); // We need to take it's inverse
        Eigen::Matrix4d tcp_T_ff = Eigen::Matrix4d::Identity();
        // Inverse
        tcp_T_ff.block(0,0,3,3)  = TCPFrame.block(0,0,3,3).transpose();
        tcp_T_ff.block(0,3,3,1) = -TCPFrame.block(0,0,3,3).transpose()*TCPFrame.block(0,3,3,1);
        // Inverse done
        tcp_T_ff.block(0,0,3,1) /= tcp_T_ff.block(0,0,3,1).norm();
        tcp_T_ff.block(0,1,3,1) /= tcp_T_ff.block(0,1,3,1).norm();
        tcp_T_ff.block(0,2,3,1) /= tcp_T_ff.block(0,2,3,1).norm();

        // Transform target to flange
        target_robBase = target_robBase*tcp_T_ff;

        target.segment(3,3) = target_robBase.block(0,0,3,1);
        target.segment(6,3) = target_robBase.block(0,1,3,1);
        target.segment(9,3) = target_robBase.block(0,2,3,1);
        target.segment(0,3) = target_robBase.block(0,3,3,1);

        // Make the Cartesian Axes Perpendicular. Evaluate // by = bz x bx and // bx = by x bz
        // by
        target(6) = (target(10)*target(5)) - (target(11)*target(4));
        target(7) = (target(11)*target(3)) - (target(9)*target(5));
        target(8) = (target(9)*target(4)) - (target(10)*target(3));
        // bx
        target(3) = (target(7)*target(11)) - (target(8)*target(10));
        target(4) = (target(8)*target(9)) - (target(6)*target(11));
        target(5) = (target(6)*target(10)) - (target(7)*target(9));
        target.segment(3,3) /= target.segment(3,3).norm(); //bx
        target.segment(9,3) /= target.segment(9,3).norm(); //by
        target.segment(6,3) /= target.segment(6,3).norm(); //bz

        Eigen::MatrixXd sol_mat;
        ik_analytical::compute_IK(target,status,sol_mat);
        if (!status)
            return status;
        if (urIKPatch)
            apply_URikPatch(sol_mat);
        
        // std::cout<< "\n\n" << sol_mat << "\n\n";
        status = false; // Make status false again so we can check if solutions exist
        double config_dist = std::numeric_limits<double>::infinity();
        int ctr = 0;
        for (int i=0; i<sol_mat.rows(); ++i){
            // Check the Limits
            if ((sol_mat(i,0)>=jt_ll(0) && sol_mat(i,0)<=jt_ul(0)) &&
                (sol_mat(i,1)>=jt_ll(1) && sol_mat(i,1)<=jt_ul(1)) &&
                (sol_mat(i,2)>=jt_ll(2) && sol_mat(i,2)<=jt_ul(2)) &&
                (sol_mat(i,3)>=jt_ll(3) && sol_mat(i,3)<=jt_ul(3)) &&
                (sol_mat(i,4)>=jt_ll(4) && sol_mat(i,4)<=jt_ul(4)) &&
                (sol_mat(i,5)>=jt_ll(5) && sol_mat(i,5)<=jt_ul(5))) 
            {
                // Add it to solution. Also check if this is closest to given config
                double dist = (sol_mat.row(i).transpose() - init_guess).norm();
                if (dist < config_dist){
                    closest_sol << sol_mat.row(i).transpose();
                    config_dist = dist;
                }
                solution.conservativeResize(OptVarDim,ctr+1);
                solution.col(ctr) = sol_mat.row(i).transpose(); ctr++;
                status = true;
            }
        }
        return status;
    }
}














// // Different forms of inputs
// // xyz and Euler angles
// bool ikHandler::solveIK(Eigen::VectorXd _target, std::string seq){
//     Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
//     for (int i=0; i<3; ++i)
//     {
//         if(seq[i]=='X' || seq[i]=='x')
//             R = R * Eigen::AngleAxisd(_target(i), Eigen::Vector3d::UnitX());
//         else if(seq[i]=='Y' || seq[i]=='y')
//             R = R * Eigen::AngleAxisd(_target(i), Eigen::Vector3d::UnitY());           
//         else if(seq[i]=='Z' || seq[i]=='z')
//             R = R * Eigen::AngleAxisd(_target(i), Eigen::Vector3d::UnitZ());                   
//     }
//     mapped_target.segment(0,3) = _target.segment(0,3);
//     mapped_target(3) = R(0,0);
//     mapped_target(4) = R(1,0);
//     mapped_target(5) = R(2,0);
//     mapped_target(6) = R(0,1);
//     mapped_target(7) = R(1,1);
//     mapped_target(8) = R(2,1);
//     mapped_target(9) = R(0,2);
//     mapped_target(10) = R(1,2);
//     mapped_target(11) = R(2,2);
//     return solveIK(mapped_target);
// };

// // Tf matrix Xdouble
// bool ikHandler::solveIK(Eigen::MatrixXd _target){
//     mapped_target.segment(0,3) = _target.block(0,3,3,1);
//     mapped_target.segment(3,3) = _target.block(0,0,3,1);
//     mapped_target.segment(6,3) = _target.block(0,1,3,1);
//     mapped_target.segment(9,3) = _target.block(0,2,3,1);
//     return solveIK(mapped_target);
// };

// // Tf matrix 4double
// bool ikHandler::solveIK(Eigen::Matrix4d _target){
//     mapped_target.segment(0,3) = _target.block(0,3,3,1);
//     mapped_target.segment(3,3) = _target.block(0,0,3,1);
//     mapped_target.segment(6,3) = _target.block(0,1,3,1);
//     mapped_target.segment(9,3) = _target.block(0,2,3,1);
//     return solveIK(mapped_target);
// };

// // Tf matrix Xfloat
// bool ikHandler::solveIK(Eigen::MatrixXf _target){
//     Eigen::MatrixXd tf = _target.cast<double>();
//     return solveIK(tf);
// };

// // Tf matrix 4float
// bool ikHandler::solveIK(Eigen::Matrix4f _target){
//     Eigen::Matrix4d tf = _target.cast<double>();
//     return solveIK(tf);
// };