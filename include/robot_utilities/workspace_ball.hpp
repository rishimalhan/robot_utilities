// author: akabir@usc.edu
// source: http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/Rickert-14-TRO.pdf

#ifndef WSB_H
#define WSB_H

#include <Eigen/Eigen>

namespace WSB{

class WSB
{
    private:


        
    public:

        int id;
        int parent_id;
        int tree_id;
        
        Eigen::MatrixXd c; // center
        double r; // radius
        Eigen::MatrixXd surface_points; 

        double gcost;
        double hcost;
        double fcost;
        
        bool is_leaf;
        bool is_root;
        
        std::vector<int> child_ids;
        
        std::vector<int> path_to_root;
                
        WSB();
        ~WSB();
        void set_id(int in_id);
        void set_parent_id(int in_parent_id);
        void set_tree_id(int in_id);

        void set_center(Eigen::MatrixXd in_c);
        void set_radius(double in_r);
        void append_surface_points(Eigen::MatrixXd sp);

        void set_cost(double in_gcost, double in_hcost, double in_fcost);

        void set_is_leaf(bool in_is_leaf);
        void set_is_root(bool in_is_root);

        void append_child_id(int in_id);
        void set_path_to_root(std::vector<int> prev_path_to_root, int in_id);
};

}
#endif        