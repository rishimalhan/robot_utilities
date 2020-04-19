#include <robot_utilities/STLtoPCL.hpp>
#include <robot_utilities/transformation_utilities.hpp>
#include <robot_utilities/file_rw.hpp>
namespace STLtoPCL{
// int main()
// {
// 	Eigen::MatrixXd v, f, n;
// 	std::string path = "/home/ak/stl_pointcloud_generation/data/";
// 	std::string stl_file_name = path + "test1.STL";
// 	igl::readSTL(stl_file_name, v, f, n);
// 	f = f.array()+1;	

// 	// std::cout << f << std::endl;
// 	int xgap = 2; // mm since stl in mm.
// 	int ygap = 2; // mm since stl in mm.

// 	Eigen::MatrixXd pts = generate_pointcloud(v,f,n,xgap,ygap);
// 	// std::cout << pts << std::endl;
// 	std::string file = path + "pts.csv";
// 	file_write(file,pts);
// 	std::cout << "file written" << std::endl;
//     return 0;
// }

Eigen::MatrixXd gen_PCL_from_STL(std::string stl_file_name, int xgap, int ygap){ 
    // all distance units in mm
    
    Eigen::MatrixXd v, f, n;

    if( (stl_file_name.find(".stl")!= std::string::npos) ||
        (stl_file_name.find(".STL")!= std::string::npos)){
        igl::readSTL(stl_file_name, v, f, n);    
        f = f.array()+1;   
        v = v.array()*1000; 
        Eigen::MatrixXd pts = generate_pointcloud(v,f,n,xgap,ygap);

        Eigen::MatrixXd pts_xyz = pts.block(0,0,pts.rows(),3)/1000.0;
        Eigen::MatrixXd pts_n = pts.block(0,3,pts.rows(),3);
        Eigen::MatrixXd pts2; 
        pts2.resize(pts.rows(),pts.cols());
        pts2 << pts_xyz,pts_n;
    
    return pts2;
    }

    if( (stl_file_name.find(".obj")!= std::string::npos) ||
        (stl_file_name.find(".OBJ")!= std::string::npos)){
        igl::readOBJ(stl_file_name, v, f);    
        f = f.array()+1;   
        v = v.array()*1000; 
        Eigen::MatrixXd pts = generate_pointcloud(v,f,xgap,ygap);    
        
        Eigen::MatrixXd pts_xyz = pts.block(0,0,pts.rows(),3)/1000.0;
        Eigen::MatrixXd pts_n = pts.block(0,3,pts.rows(),3);
        Eigen::MatrixXd pts2; 
        pts2.resize(pts.rows(),pts.cols());
        pts2 << pts_xyz,pts_n;

    return pts2;
    }
    
    // f = f.array()+1;    
    // Eigen::MatrixXd pts = generate_pointcloud(v,f,n,xgap,ygap);
    // std::string file = "../data/world_point_cloud.csv";
    // STLtoPCL::file_write(file,pts);
    // std::cout << "world point cloud file written to: " << file <<std::endl;  
    // return pts;
}

Eigen::MatrixXd gen_positions(Eigen::MatrixXd in_pts){ 
    // assumes original points are in mm
    // returns in meters

    // std::string file = "../data/world_point_cloud.csv";
    // STLtoPCL::file_write(file,in_pts.block(0,0,in_pts.rows(),3)/1000.0);
    // std::cout << "world point cloud file written to: " << file <<std::endl;  
    
    return in_pts.block(0,0,in_pts.rows(),3)/1000.0;    
}

void file_write(std::string file_name, const Eigen::MatrixXd& mat)
    {
        std::ofstream out_file;
        out_file.open(file_name.c_str(),std::ios::out);
        for (long i=0;i<mat.rows();++i)
        {
            for (long j=0;j<mat.cols();++j)
            {
                if (j!=mat.cols()-1)
                {
                    out_file << mat(i,j) << ",";    
                }
                else
                {
                    out_file << mat(i,j) << std::endl;       
                }
            }
        }
        out_file.close();
    }


// Eigen::MatrixXd generate_pointcloud(const Eigen::MatrixXd& v, const Eigen::MatrixXd& f, const Eigen::MatrixXd& n, double gap_x, double gap_y)
Eigen::MatrixXd generate_pointcloud(Eigen::MatrixXd v, Eigen::MatrixXd f, Eigen::MatrixXd n, double gap_x, double gap_y)
{
    long max_pts_size = 100000000;
    Eigen::MatrixXd ptcloud_w_normals(max_pts_size,6);
    // Eigen::MatrixXd ptcloud_normals;
    long idx_start = 0;
    double alpha, beta;
    Eigen::MatrixXd tri(3,3);
    Eigen::MatrixXd tri2(3,3);
    Eigen::MatrixXd tri3(3,3);
    Eigen::MatrixXd tri4(3,3);
    Eigen::MatrixXd nn(1,3);
    Eigen::MatrixXd nn2(1,3);
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4); 
    Eigen::MatrixXd T2 = Eigen::MatrixXd::Identity(4,4); 

    for (long idx=0;idx<f.rows();++idx)
    {
        tri.row(0) << v.row(f(idx,0)-1);
        tri.row(1) << v.row(f(idx,1)-1);
        tri.row(2) << v.row(f(idx,2)-1);
        nn = n.row(idx);
        tri2.row(0) << 0,0,0;
        tri2.row(1) << tri.row(1).array() - tri.row(0).array();
        tri2.row(2) << tri.row(2).array() - tri.row(0).array();
        if (nn(0,1)==0 && nn(0,2)==0)
        {
            alpha = 0;
        }
        else
        {
            alpha = atan(nn(0,1)/nn(0,2));
        }

        T.block(0,0,3,3) = rot_x(alpha);
        nn2 = apply_transformation(nn,T);
        tri3 = apply_transformation(tri2,T);
        if (nn2(0,0)==0 && nn2(0,2)==0)
        {
            beta = 0;
        }
        else
        {
            beta = -atan(nn2(0,0)/nn2(0,2));
        }
        T2.block(0,0,3,3) = rot_y(beta); 
        tri4 = apply_transformation(tri3,T2);
        Eigen::MatrixXd grid_pts = generate_grid_points(gap_x, gap_y, tri4.col(0).minCoeff(), tri4.col(1).minCoeff(), tri4.col(0).maxCoeff(), tri4.col(1).maxCoeff());
        Eigen::MatrixXd pts = add_pts(tri4, grid_pts);
        if (pts.rows()!=0)
        {
            Eigen::MatrixXd pts2 = apply_transformation(pts,T2.inverse());
            Eigen::MatrixXd pts3 = apply_transformation(pts2,T.inverse());
            pts3.col(0) = pts3.col(0).array() + tri(0,0);
            pts3.col(1) = pts3.col(1).array() + tri(0,1);
            pts3.col(2) = pts3.col(2).array() + tri(0,2);

            if (idx_start+pts3.rows() < max_pts_size)
            {
                ptcloud_w_normals.block(idx_start,0,pts3.rows(),3) = pts3;
                Eigen::MatrixXd nn_mat = Eigen::MatrixXd::Constant(pts3.rows(),3,1);
                nn_mat.col(0) = nn_mat.col(0).array()*nn(0,0);
                nn_mat.col(1) = nn_mat.col(1).array()*nn(0,1);
                nn_mat.col(2) = nn_mat.col(2).array()*nn(0,2);
                ptcloud_w_normals.block(idx_start,3,pts3.rows(),3) = nn_mat;
                idx_start = idx_start + pts3.rows();
            }
            else
            {
                std::cout << "ERROR : Pointcloud size limit exceeded!" << std::endl;
                return ptcloud_w_normals.block(0,0,--idx_start,6);    
            }
        }
    }
    Eigen::MatrixXd bug = ptcloud_w_normals.block(0,0,idx_start,6);
    file_rw::file_write("../data/normals_bug_1.csv", bug);
    return ptcloud_w_normals.block(0,0,idx_start,6);
}

// Eigen::MatrixXd generate_pointcloud(Eigen::MatrixXd v, Eigen::MatrixXd f, Eigen::MatrixXd n, double gap_x, double gap_y)
//     {
//         long max_pts_size = 100000000;
//         Eigen::MatrixXd ptcloud_w_normals(max_pts_size,6);
//         // Eigen::MatrixXd ptcloud_normals;
//         long idx_start = 0;
//         double alpha, beta;
//         Eigen::MatrixXd tri(3,3);
//         Eigen::MatrixXd tri2(3,3);
//         Eigen::MatrixXd tri3(3,3);
//         Eigen::MatrixXd tri4(3,3);
//         Eigen::MatrixXd nn(1,3);
//         Eigen::MatrixXd nn2(1,3);
//         Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4); 
//         Eigen::MatrixXd T2 = Eigen::MatrixXd::Identity(4,4); 
//         Eigen::MatrixXd T2_inv(4,4); 
//         Eigen::MatrixXd T_inv(4,4);    

//         for (long idx=0;idx<f.rows();++idx)
//         {
//             tri.row(0) << v.row(f(idx,0)-1);
//             tri.row(1) << v.row(f(idx,1)-1);
//             tri.row(2) << v.row(f(idx,2)-1);

//             // tri.row(0) << v.row(f(idx,0)-1) * 1000.0;
//             // tri.row(1) << v.row(f(idx,1)-1) * 1000.0;
//             // tri.row(2) << v.row(f(idx,2)-1) * 1000.0;            

//             nn = n.row(idx);
//             tri2.row(0) << 0,0,0;
//             tri2.row(1) << tri.row(1).array() - tri.row(0).array();
//             tri2.row(2) << tri.row(2).array() - tri.row(0).array();
//             if (nn(0,1)==0 && nn(0,2)==0)
//             {
//                 alpha = 0;
//             }
//             else
//             {
//                 alpha = atan(nn(0,1)/nn(0,2));
//             }

//             T.block(0,0,3,3) = rot_x(alpha);
//             nn2 = apply_transformation(nn,T);
//             tri3 = apply_transformation(tri2,T);
//             if (nn2(0,0)==0 && nn2(0,2)==0)
//             {
//                 beta = 0;
//             }
//             else
//             {
//                 beta = -atan(nn2(0,0)/nn2(0,2));
//             }
//             T2.block(0,0,3,3) = rot_y(beta); 
//             tri4 = apply_transformation(tri3,T2);
//             Eigen::MatrixXd grid_pts = generate_grid_points(gap_x, gap_y, tri4.col(0).minCoeff(), tri4.col(1).minCoeff(), tri4.col(0).maxCoeff(), tri4.col(1).maxCoeff());
//             Eigen::MatrixXd pts = add_pts(tri4, grid_pts);
//             if (pts.rows()!=0)
//             {
//             	T2_inv = T2.inverse();
//             	T_inv = T.inverse();
//                 Eigen::MatrixXd pts2 = apply_transformation(pts,T2_inv);
//                 Eigen::MatrixXd pts3 = apply_transformation(pts2,T_inv);
//                 pts3.col(0) = pts3.col(0).array() + tri(0,0);
//                 pts3.col(1) = pts3.col(1).array() + tri(0,1);
//                 pts3.col(2) = pts3.col(2).array() + tri(0,2);

//                 if (idx_start+pts3.rows() < max_pts_size)
//                 {
//                     ptcloud_w_normals.block(idx_start,0,pts3.rows(),3) = pts3;
//                     Eigen::MatrixXd nn_mat = Eigen::MatrixXd::Constant(pts3.rows(),3,1);
//                     nn_mat.col(0) = nn_mat.col(0).array()*nn(0,0);
//                     nn_mat.col(1) = nn_mat.col(1).array()*nn(0,1);
//                     nn_mat.col(2) = nn_mat.col(2).array()*nn(0,2);
//                     ptcloud_w_normals.block(idx_start,3,pts3.rows(),3) = nn_mat;
//                     idx_start = idx_start + pts3.rows();
//                 }
//                 else
//                 {
//                     std::cout << "ERROR : Pointcloud size limit exceeded!" << std::endl;
//                     return ptcloud_w_normals.block(0,0,--idx_start,6);    
//                 }
//             }
//         }
//         return ptcloud_w_normals.block(0,0,idx_start,6);
//     }

Eigen::MatrixXd generate_pointcloud(Eigen::MatrixXd v, Eigen::MatrixXd f, double gap_x, double gap_y)
    {
        long max_pts_size = 100000000;
        Eigen::MatrixXd ptcloud_w_normals(max_pts_size,6);
        // Eigen::MatrixXd ptcloud_normals;
        long idx_start = 0;
        double alpha, beta;
        Eigen::MatrixXd tri(3,3);
        Eigen::MatrixXd tri2(3,3);
        Eigen::MatrixXd tri3(3,3);
        Eigen::MatrixXd tri4(3,3);
        Eigen::MatrixXd nn(1,3);
        Eigen::MatrixXd nn2(1,3);
        Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4); 
        Eigen::MatrixXd T2 = Eigen::MatrixXd::Identity(4,4); 
        Eigen::MatrixXd T2_inv(4,4); 
        Eigen::MatrixXd T_inv(4,4);    

        for (long idx=0;idx<f.rows();++idx)
        {
            // tri.row(0) << v.row(f(idx,0)-1);
            // tri.row(1) << v.row(f(idx,1)-1);
            // tri.row(2) << v.row(f(idx,2)-1);

            tri.row(0) << v.row(f(idx,0)-1) * 1000.0;
            tri.row(1) << v.row(f(idx,1)-1) * 1000.0;
            tri.row(2) << v.row(f(idx,2)-1) * 1000.0;            

            nn << 0,0,1;
            tri2.row(0) << 0,0,0;
            tri2.row(1) << tri.row(1).array() - tri.row(0).array();
            tri2.row(2) << tri.row(2).array() - tri.row(0).array();
            if (nn(0,1)==0 && nn(0,2)==0)
            {
                alpha = 0;
            }
            else
            {
                alpha = atan(nn(0,1)/nn(0,2));
            }

            T.block(0,0,3,3) = rot_x(alpha);
            nn2 = apply_transformation(nn,T);
            tri3 = apply_transformation(tri2,T);
            if (nn2(0,0)==0 && nn2(0,2)==0)
            {
                beta = 0;
            }
            else
            {
                beta = -atan(nn2(0,0)/nn2(0,2));
            }
            T2.block(0,0,3,3) = rot_y(beta); 
            tri4 = apply_transformation(tri3,T2);
            Eigen::MatrixXd grid_pts = generate_grid_points(gap_x, gap_y, tri4.col(0).minCoeff(), tri4.col(1).minCoeff(), tri4.col(0).maxCoeff(), tri4.col(1).maxCoeff());
            Eigen::MatrixXd pts = add_pts(tri4, grid_pts);
            if (pts.rows()!=0)
            {
                T2_inv = T2.inverse();
                T_inv = T.inverse();
                Eigen::MatrixXd pts2 = apply_transformation(pts,T2_inv);
                Eigen::MatrixXd pts3 = apply_transformation(pts2,T_inv);
                pts3.col(0) = pts3.col(0).array() + tri(0,0);
                pts3.col(1) = pts3.col(1).array() + tri(0,1);
                pts3.col(2) = pts3.col(2).array() + tri(0,2);

                if (idx_start+pts3.rows() < max_pts_size)
                {
                    ptcloud_w_normals.block(idx_start,0,pts3.rows(),3) = pts3;
                    Eigen::MatrixXd nn_mat = Eigen::MatrixXd::Constant(pts3.rows(),3,1);
                    nn_mat.col(0) = nn_mat.col(0).array()*nn(0,0);
                    nn_mat.col(1) = nn_mat.col(1).array()*nn(0,1);
                    nn_mat.col(2) = nn_mat.col(2).array()*nn(0,2);
                    ptcloud_w_normals.block(idx_start,3,pts3.rows(),3) = nn_mat;
                    idx_start = idx_start + pts3.rows();
                }
                else
                {
                    std::cout << "ERROR : Pointcloud size limit exceeded!" << std::endl;
                    return ptcloud_w_normals.block(0,0,--idx_start,6);    
                }
            }
        }
        return ptcloud_w_normals.block(0,0,idx_start,6);
    }

    bool lines_intersect(double l1[2][2], double l2[2][2])
    {
        // l1 for horizontal ray line...slope is always zero

        // checking if other slope is zero
        if (l2[0][1]==l2[1][1])
        {
            return false;
        }
        else
        {
            // checking both pts of second line above first line
            if ((l2[0][1]>l1[0][1] && l2[1][1]>l1[0][1]) || (l2[0][1]<l1[0][1] && l2[1][1]<l1[0][1]))
            {
                return false;
            }
            else
            {
                // checking both pts of second line either on right or on left of fist line
                if ((l2[0][0]<l1[0][0] && l2[1][0]<l1[0][0]) || (l2[0][0]>l1[1][0] && l2[1][0]>l1[1][0]))
                {
                    return false;
                }
                else
                {
                    // checking if other line is vertical
                    if (l2[0][0]== l2[1][0])
                    {
                        return true;
                    }
                    else
                    {
                        // getting intersection point
                        double m2 = (l2[1][1]-l2[0][1])/(l2[1][0]-l2[0][0]);        
                        double x = (l1[0][1]+m2*l2[0][0]-l2[0][1])/m2;
                        // checking if intersection point lies on the first line
                        if ((x>l1[0][0] || std::abs(x-l1[0][0])<1e-9) && (x<l1[1][0] || std::abs(x-l1[1][0])<1e-9))
                        {
                            return true;
                        }
                        else
                        {
                            return false;
                        }
                    }
                }
            }
        } 
        return false;
    }

    Eigen::Matrix3d rot_x(double t)
	{
		Eigen::Matrix3d rx;
		rx << 	1,		0,		0,
				0, cos(t),-sin(t),
				0, sin(t), cos(t);
		return rx;  
	}

    Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::MatrixXd T_mat)
	{
	    //! NOTE: Homogeneous Tranformation Matrix (4x4)

	    //! putting data in [x, y, z, 1]' format
	    Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
	    Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
	    data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
	    data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
	    Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
	    Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
	    transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
	    return transformed_data_mat.transpose();
	}

    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd add_pts(Eigen::MatrixXd tri, Eigen::MatrixXd grid_pts)
    {
        Eigen::MatrixXd in = InPoly(grid_pts, tri);
        std::vector<int> loc = find_idx(in);
        Eigen::MatrixXd pts = Eigen::MatrixXd::Constant(loc.size(),3,0);
        for (int i=0;i<loc.size();++i)
        {
            pts(i,0) = grid_pts(loc[i],0);
            pts(i,1) = grid_pts(loc[i],1);
        }
        return pts;
    }

	std::vector<int> find_idx(Eigen::VectorXi vec)
    {
        std::vector<int> idx;
        for (int i=0;i<vec.rows();++i)
        {
            if (vec(i,0)!=0)
            {
                idx.push_back(i);   
            }
        }
        return idx;
    }
    std::vector<int> find_idx(Eigen::MatrixXd vec)
    {
        std::vector<int> idx;
        for (int i=0;i<vec.rows();++i)
        {
            if (vec(i,0)!=0)
            {
                idx.push_back(i);   
            }
        }
        return idx;
    }

    Eigen::Matrix3d rot_y(double t)
	{
		Eigen::Matrix3d ry;
		ry << cos(t),	0, sin(t),
				   0,	1,		0,
			 -sin(t),	0, cos(t);
		return ry;  
	}

    ////////////////////////////////////////////////////////////

    Eigen::MatrixXd generate_grid_points(double pathgap_x, double pathgap_y, double xmin, double ymin, double xmax, double ymax)
    {
        // Function to generate the uniform mesh grid of points along the x-y plane
        // INPUT = gap between the adjacent points and maximum value in x and y direction
        // OUTPUT = All points consisting the uniform grid
        Eigen::VectorXd j = linsp(floor(ymin),ceil(ymax),pathgap_y);
        Eigen::VectorXd i_val = linsp(floor(xmin),ceil(xmax),pathgap_x);
        Eigen::MatrixXd pts = Eigen::MatrixXd::Constant(j.rows()*i_val.rows(),2,0);
        long int st_pt = 0;
        for (long int i=0;i<i_val.rows();++i)
        {
            pts.block(st_pt,0,j.rows(),1) = i_val(i)*Eigen::MatrixXd::Constant(j.rows(),1,1);
            pts.block(st_pt,1,j.rows(),1) = j.block(0,0,j.rows(),j.cols());
            st_pt = st_pt + j.rows();
        }
        return pts; 
    }

    Eigen::VectorXd linsp(double strt, double end, double stp)
    {
        int sz;
        if (strt<=end && stp>0 || strt>=end && stp<0)
        {
            sz = int((end-strt)/stp)+1;
        }
        else
        {
            if (strt>=end)
            {
                std::cerr << "start value is greater than the end value for incement!" << std::endl;
                std::terminate();   
            }
            else
            {
                std::cerr << "start value is less than the end value for decrement!" << std::endl;
                std::terminate();   
            }
        }
        return Eigen::VectorXd::LinSpaced(sz,strt,strt+stp*(sz-1));
    }

    Eigen::MatrixXd InPoly(Eigen::MatrixXd q, Eigen::MatrixXd p)
    {
        double l1[2][2];
        double l2[2][2];

        Eigen::MatrixXd in = Eigen::VectorXd::Constant(q.rows(),1,0);

        double xmin = p.col(0).minCoeff();
        double xmax = p.col(0).maxCoeff();
        double ymin = p.col(1).minCoeff();
        double ymax = p.col(1).maxCoeff();

        for (long i=0;i<q.rows();++i)
        {
            // bounding box test
            if (q(i,0)<xmin || q(i,0)>xmax || q(i,1)<ymin || q(i,1)>ymax)
            {
                continue;
            }
            int intersection_count = 0;
            Eigen::MatrixXd cont_lines = Eigen::MatrixXd::Constant(p.rows(),1,0);
            for (int j=0;j<p.rows();++j)
            {
                if (j==0)
                {
                    l1[0][0] = q(i,0);l1[0][1] = q(i,1);
                    l1[1][0] = xmax;l1[1][1] = q(i,1);
                    l2[0][0] = p(p.rows()-1,0);l2[0][1] = p(p.rows()-1,1);
                    l2[1][0] = p(j,0);l2[1][1] = p(j,1);
                    if (lines_intersect(l1,l2))
                    {
                        intersection_count++;
                        cont_lines(j,0) = 1;
                    }   
                }
                else
                {
                    l1[0][0] = q(i,0);l1[0][1] = q(i,1);
                    l1[1][0] = xmax;l1[1][1] = q(i,1);
                    l2[0][0] = p(j,0);l2[0][1] = p(j,1);
                    l2[1][0] = p(j-1,0);l2[1][1] = p(j-1,1);
                    if (lines_intersect(l1,l2))
                    {
                        intersection_count++;
                        cont_lines(j,0) = 1;
                        if (cont_lines(j-1,0)==1)
                        {
                            if (p(j-1,1)==q(i,1))
                            {
                                if (j-1==0)
                                {
                                    if (!((p(p.rows()-1,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(p.rows()-1,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
                                    {
                                        intersection_count--;
                                    }
                                }
                                else
                                {
                                    if (!((p(j-2,1)<p(j-1,1) && p(j,1)<p(j-1,1)) || (p(j-2,1)>p(j-1,1) && p(j,1)>p(j-1,1))))
                                    {
                                        intersection_count--;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (intersection_count%2==1)
            {
                in(i,0) = 1;
            }
        }
        return in;
    }
}