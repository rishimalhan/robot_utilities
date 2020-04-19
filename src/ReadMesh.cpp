# include <robot_utilities/ReadMesh.hpp>

namespace ReadMesh{

	void loadModel(const char* filename, std::vector<fcl::Vector3<double>>& points, std::vector<fcl::Triangle>& triangles)
	{
	  std::string the_name = filename;
	  if (the_name.find(".STL") != std::string::npos || the_name.find(".stl") != std::string::npos)
	  {
	    loadSTLFile(filename, points, triangles);
	  }
	  else if (the_name.find(".OBJ") != std::string::npos || the_name.find(".obj") != std::string::npos)
	  {
	    loadOBJFile(filename, points, triangles);
	  }
	}

	void loadSTLFile(const char* filename, std::vector<fcl::Vector3<double>>& points, std::vector<fcl::Triangle>& triangles)
	{
		Eigen::MatrixXd v, f, n;
		std::string _filename = filename;
		igl::readSTL(_filename, v, f, n);
		int vc = v.cols();
		int fc = f.cols();
		long int vr = v.rows();
		long int fr = f.rows();
		if ((vc != 3)||(fc != 3)) 
		{
			std::cerr<<"this STL file has wrong format!"<<std::endl;
			return;
		}else
		{
			fcl::Vector3<double> vertex;
			fcl::Triangle tri;
			for(long int i = 0; i < vr; ++i)
			{
				
				vertex << v(i, 0), v(i,1), v(i,2);
				points.push_back(vertex);
			}
			for(long int j = 0; j < fr; ++j)
			{
				tri[0] = f(j, 0);
				tri[1] = f(j, 1);
				tri[2] = f(j, 2);
				triangles.push_back(tri);
			}
		}
	}

	void loadOBJFile(const char* filename, std::vector<fcl::Vector3<double>>& points, std::vector<fcl::Triangle>& triangles)
	{
		Eigen::MatrixXd v, f;
		std::string _filename = filename;
		igl::readOBJ(_filename, v, f);
		int vc = v.cols();
		int fc = f.cols();
		long int vr = v.rows();
		long int fr = f.rows();
		if ((vc != 3)||(fc != 3)) 
		{
			std::cerr<<"this STL file has wrong format!"<<std::endl;
			return;
		}else
		{
			fcl::Vector3<double> vertex;
			fcl::Triangle tri;
			for(long int i = 0; i < vr; ++i)
			{
				
				vertex << v(i, 0), v(i,1), v(i,2);
				points.push_back(vertex);
			}
			for(long int j = 0; j < fr; ++j)
			{
				tri[0] = f(j, 0);
				tri[1] = f(j, 1);
				tri[2] = f(j, 2);
				triangles.push_back(tri);
			}
		}
	}
}