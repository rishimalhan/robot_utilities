// author: akabir@usc.edu
// source: http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/Rickert-14-TRO.pdf

#ifndef WSBM_H
#define WSBM_H

#include <math.h>
#include<random>
#include<cmath>
#include<chrono>
#include<vector>
#include<algorithm>
#include<iostream>

#include <Eigen/Eigen>
#include "nabo/nabo.h"

// #include <file_rw.hpp>
#include <robot_utilities/priority_queue.hpp>
#include <robot_utilities/workspace_ball.hpp>

namespace WSBM{

class WSBM
{
    private:

    public:
        Eigen::MatrixXd M;
        priorityQ::PriorityQueue ballsPQ;
        std::vector<WSB::WSB> all_balls;
        std::vector<int> kill_balls_idx;
        int ball_counter;

        Eigen::MatrixXd start_p;
        Eigen::MatrixXd goal_p;
        Nabo::NNSearchD * nns;

        int surface_points_N;

        std::vector<int> ball_sequence;

        Eigen::MatrixXd ball_sequence_c_r;

        double obstacle_epsilon;
        double MAX_radius;

        WSBM(Eigen::MatrixXd in_start_p, Eigen::MatrixXd in_goal_p, Eigen::MatrixXd world_pts);
        ~WSBM();
        double get_distance_to_nearest_obstacle(Eigen::MatrixXd p);
        WSB::WSB create_WB(Eigen::MatrixXd center, int parent_id);
        double distance_to_goal(WSB::WSB ball);
        Eigen::MatrixXd generate_random_points_on_unit_sphere(int N);
        void generate_surface_points(WSB::WSB * ball);
        void create_WB_tree();
};

}
#endif        