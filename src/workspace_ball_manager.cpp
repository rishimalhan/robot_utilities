// author: akabir@usc.edu
// source: http://www.robotics.tu-berlin.de/fileadmin/fg170/Publikationen_pdf/Rickert-14-TRO.pdf

#include <robot_utilities/workspace_ball_manager.hpp>
#include <robot_utilities/file_rw.hpp>

namespace WSBM{

    WSBM::WSBM(Eigen::MatrixXd in_start_p, Eigen::MatrixXd in_goal_p, Eigen::MatrixXd world_pts){
        ball_counter = 0;
        start_p.resize(in_start_p.rows(),in_start_p.cols());
        goal_p.resize(in_start_p.rows(),in_start_p.cols());
        start_p = in_start_p;
        goal_p = in_goal_p;
        
        // file_rw::file_write("/home/aniruddha/random_data/WSB_s.csv", start_p);
        // file_rw::file_write("/home/aniruddha/random_data/WSB_g.csv", goal_p);
        // file_rw::file_write("../WSB_s.csv", start_p);
        // file_rw::file_write("../WSB_g.csv", goal_p);

        M.resize(world_pts.cols(),world_pts.rows());
        M = world_pts.transpose();
        nns = Nabo::NNSearchD::createKDTreeLinearHeap(M);

        all_balls.clear();
        kill_balls_idx.clear();

        surface_points_N = 10;
        ball_sequence.clear();

        obstacle_epsilon = 0.01;
        MAX_radius = 0.25;
    };

    WSBM::~WSBM(){};

    double WSBM::get_distance_to_nearest_obstacle(Eigen::MatrixXd p){
        const int K = 1;
        Eigen::VectorXi indices(K);
        Eigen::VectorXd dists2(K);
        nns->knn(p, indices, dists2, K);
        // return dists2(0);
        return sqrt(dists2(0));
    }


    WSB::WSB WSBM::create_WB(Eigen::MatrixXd center, int parent_id){
        WSB::WSB ball;
        double g_cost,h_cost;

        ball.set_id(ball_counter++);
        ball.set_parent_id(parent_id);
        ball.set_center(center);    

        double ball_rad = std::max(get_distance_to_nearest_obstacle(ball.c)*0.9, MAX_radius);  
        ball.set_radius(ball_rad); 

        if(parent_id > -1){
            all_balls[parent_id].append_child_id(ball.id);
            g_cost = all_balls[parent_id].gcost + all_balls[parent_id].r;            
        }
        else{
            g_cost = 0;               
        }

        h_cost = distance_to_goal(ball);
        ball.set_cost(g_cost, h_cost, g_cost+h_cost);
        
        generate_surface_points(&ball);
        
        all_balls.push_back(ball);

        Eigen::MatrixXd temp_ball;
        temp_ball.resize(4,all_balls.size());
        for (uint i = 0; i < all_balls.size(); ++i){
            temp_ball(0,i) = all_balls[i].c(0,0);
            temp_ball(1,i) = all_balls[i].c(1,0);
            temp_ball(2,i) = all_balls[i].c(2,0);
            temp_ball(3,i) = all_balls[i].r;
        }
        // file_rw::file_write("/home/aniruddha/random_data/all_balls_c_r.csv", temp_ball);
        // file_rw::file_write("../all_balls_c_r.csv", temp_ball);


        return ball;
    };

    double WSBM::distance_to_goal(WSB::WSB ball){
        return abs((ball.c - goal_p).norm() - ball.r);
    };
    
    Eigen::MatrixXd WSBM::generate_random_points_on_unit_sphere(int N){
        Eigen::MatrixXd rand_points (3,N);
        // Set up random number generators
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::mt19937 generator (seed);
        std::uniform_real_distribution<double> uniform01(0.0, 1.0);        
        double theta,phi,x,y,z;

        for (uint i = 0; i<N; ++i){
            theta = 2 * M_PI * uniform01(generator);
            phi = acos(1 - 2 * uniform01(generator));
            rand_points(0,i) = sin(phi) * cos(theta);
            rand_points(1,i)  = sin(phi) * sin(theta);
            rand_points(2,i)  = cos(phi);                    
        }
        return rand_points;
    };

    void WSBM::generate_surface_points(WSB::WSB * ball){
        // Eigen::MatrixXd surf_points_unit = generate_random_points_on_unit_sphere(surface_points_N);
        // Eigen::MatrixXd surf_points(ball->c.rows(),surface_points_N);
        // for (uint i = 0; i<ball->c.rows(); ++i){
        //     surf_points.row(i) = surf_points_unit.row(i).array() + ball->c(i,0);
        // }


        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
        std::mt19937 generator (seed);
        std::uniform_real_distribution<double> uniform01(-1.0, 1.0);
        double x,y,z;  

        Eigen::MatrixXd surf_points(ball->c.rows(),surface_points_N);
        for (uint i = 0; i<surface_points_N; ++i){
            x = uniform01(generator);
            y = uniform01(generator);
            z = uniform01(generator);
            Eigen::Vector3d v(x,y,z);
            Eigen::Vector3d vn = v.normalized();

            surf_points(0,i) = ball->c(0,0) + ball->r * vn(0);
            surf_points(1,i) = ball->c(1,0) + ball->r * vn(1);
            surf_points(2,i) = ball->c(2,0) + ball->r * vn(2);
        }


        ball->append_surface_points(surf_points);
    };
    
    void WSBM::create_WB_tree(){
        WSB::WSB ball = create_WB(start_p, -1);
        ballsPQ.push(ball.id, ball.gcost, ball.fcost);

        int current_ball_id;
        WSB::WSB current_ball;

        int goal_ball_id;
        WSB::WSB goal_ball;
        bool solution_found = false;

        int iter = 0;
        while(ballsPQ.size()>0){

            // std::cout << "wsb gen iterator: " << iter++ << std::endl;

            current_ball_id = ballsPQ.top();
            current_ball = all_balls[current_ball_id];

            ballsPQ.pop();
            kill_balls_idx.push_back(current_ball_id);


            // std::cout << "current_ball_id: " << current_ball_id << ", total balls: "<< ball_counter<< std::endl;

            
            // if(distance_to_goal(current_ball)<current_ball.r){
            if((current_ball.c - goal_p).norm()<current_ball.r){                


                // std::cout << "center: " << current_ball.c(0,0) << ", " << current_ball.c(1,0) << ", " << current_ball.c(2,0) << std::endl;
                // std::cout << "radius: " << current_ball.r << std::endl;
                // std::cout << "distance to goal: " << abs((current_ball.c - goal_p).norm() - current_ball.r) << std::endl;

                // GOAL REACHED
                goal_ball_id = current_ball_id;
                goal_ball = current_ball;
                solution_found = true;
                
                // std::cout << "current_ball_id: " << current_ball_id << std::endl;
                // std::cout << "goal_ball_id: " << goal_ball_id << std::endl;

                // std::cout << "current_ball_parent_id: " << current_ball.parent_id << std::endl;
                // std::cout << "goal_ball_parent_id: " << goal_ball.parent_id << std::endl;

                // std::cout << "current_ball_own_id: " << current_ball.id << std::endl;
                // std::cout << "goal_ball_own_id: " << goal_ball.id << std::endl;

                break;
            }

            bool surface_point_in_existing_ball;

            for(uint i=0; i<current_ball.surface_points.cols(); ++i){ // for each surface point on current ball
                Eigen::MatrixXd sp = current_ball.surface_points.block(0,i,3,1); // surface point on current ball
                surface_point_in_existing_ball = false;    
                for (uint j = 0; j<kill_balls_idx.size(); ++j ){ // for each sphere in the tree

                    if(current_ball_id == all_balls[kill_balls_idx[j]].id) // skip current ball
                        continue;
                    

                    // if( std::find(kill_balls_idx.begin(),kill_balls_idx.end(),all_balls[j].id) ) // skip balls that are not in open list
                    //     continue;

                    if ((all_balls[kill_balls_idx[j]].c - sp).norm() < all_balls[kill_balls_idx[j]].r){ // if current surface point in ball
                        surface_point_in_existing_ball = true;
                        break;
                    }

                    if(get_distance_to_nearest_obstacle(sp) < obstacle_epsilon){ // if too close to obstacle
                        surface_point_in_existing_ball = true;
                        break;
                    }

                }
                if(!surface_point_in_existing_ball){ // current surface point does not belong to a ball
                    WSB::WSB child_ball = create_WB(sp, current_ball_id);
                    // ballsPQ.push(child_ball.id, child_ball.gcost, child_ball.fcost);
                    ballsPQ.push(child_ball.id, child_ball.gcost, child_ball.hcost);
                }
            }
        }

        if(solution_found){ // sequence balls connecting start and goal
            // std::cout << "WSB Gen Completed Successfully" << std::endl;
            std::vector<int> temp; 
            temp.clear();
            current_ball = goal_ball;
            temp.push_back(current_ball.id);
            // std::cout << "current_ball_id: " << current_ball.id << ", " << temp[temp.size()-1] << std::endl;                
            while(current_ball.parent_id>-1){
                current_ball = all_balls[current_ball.parent_id];
                temp.push_back(current_ball.id);
                // std::cout << "current_ball_id: " << current_ball.id << ", " << temp[temp.size()-1] << std::endl;                
            }
            

            ball_sequence_c_r.resize(4,temp.size());
            for (uint i = 0; i < temp.size(); ++i){
                ball_sequence.push_back(temp[temp.size()-1-i]);
                
                ball_sequence_c_r(0,i) = all_balls[temp[temp.size()-1-i]].c(0,0);
                ball_sequence_c_r(1,i) = all_balls[temp[temp.size()-1-i]].c(1,0);
                ball_sequence_c_r(2,i) = all_balls[temp[temp.size()-1-i]].c(2,0);
                ball_sequence_c_r(3,i) = all_balls[temp[temp.size()-1-i]].r;
                // std::cout << i << ", " 
                //           << temp.size()-1-i << ", " 
                //           << temp[temp.size()-1-i] << ", " 
                //           << all_balls[temp[temp.size()-1-i]].id 
                //           << std::endl;
            }
            // file_rw::file_write("/home/aniruddha/random_data/ball_sequence_c_r.csv", ball_sequence_c_r);
            file_rw::file_write("../data/ball_sequence_c_r.csv", ball_sequence_c_r);
            // std::cout << "# of WSB: "<< temp.size() << std::endl;

        }

        
        ball_sequence_c_r.resize(4,all_balls.size());
        for (uint i = 0; i < all_balls.size(); ++i){
            ball_sequence_c_r(0,i) = all_balls[i].c(0,0);
            ball_sequence_c_r(1,i) = all_balls[i].c(1,0);
            ball_sequence_c_r(2,i) = all_balls[i].c(2,0);
            ball_sequence_c_r(3,i) = all_balls[i].r;
        }
        // file_rw::file_write("/home/aniruddha/random_data/all_balls_c_r.csv", ball_sequence_c_r);
        // file_rw::file_write("../all_balls_c_r.csv", ball_sequence_c_r);
        


    };    
        

}