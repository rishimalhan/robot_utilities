// author: akabir@usc.edu
// source: http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/Rickert-14-TRO.pdf

#include <robot_utilities/workspace_ball.hpp>

namespace WSB{

    WSB::WSB(){
        child_ids.clear();
        path_to_root.clear();
        surface_points.resize(3,0);
    };

    WSB::~WSB(){};

    void WSB::set_id(int in_id){
        id = in_id;
    };
    
    void WSB::set_parent_id(int in_parent_id){
        parent_id = in_parent_id;
    };
    
    void WSB::set_tree_id(int in_tree_id){
        tree_id = in_tree_id;
    };
    

    void WSB::set_center(Eigen::MatrixXd in_c){
        c.resize(in_c.rows(),in_c.cols());
        c = in_c;
    };
    
    void WSB::set_radius(double in_r){
        r = in_r;
    };
    
    void WSB::append_surface_points(Eigen::MatrixXd sp){
        Eigen::MatrixXd tmp(3, surface_points.cols()+sp.cols());
        tmp << surface_points, sp;
        surface_points = tmp;
    };
    

    void WSB::set_cost(double in_gcost, double in_hcost, double in_fcost){
        gcost = in_gcost;
        hcost = in_hcost;
        fcost = in_fcost;
    };
    

    void WSB::set_is_leaf(bool in_is_leaf){

    };
    
    void WSB::set_is_root(bool in_is_root){

    };
    

    void WSB::append_child_id(int in_id){
        child_ids.push_back(in_id);
    };
    
    void WSB::set_path_to_root(std::vector<int> prev_path_to_root, int in_id){

    };
    

}