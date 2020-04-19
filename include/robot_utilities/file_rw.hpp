#ifndef FILE_RW
#define FILE_RW

#include <string>
#include <vector>
#include <Eigen/Eigen>

class file_rw
{
public:
    // read data from file and store in vector
    static std::vector< std::vector<double> > file_read_vec(std::string);

	// read data from file and store in matrix
    static Eigen::MatrixXd file_read_mat(std::string);

    // write data to file from vector
    static void file_write(std::string, const std::vector< std::vector<int> >&);
    static void file_write(std::string, const std::vector< std::vector<float> >&);
    static void file_write(std::string, const std::vector< std::vector<double> >&);
    
	// write data to file from matrix
    static void file_write(std::string, const Eigen::MatrixXd&);
	static void file_write(std::string, const Eigen::MatrixXi&);
    static void file_write(std::string, const Eigen::MatrixXf&);
    static void file_write(std::string, const std::vector< std::string >&);
    static std::vector<std::string> file_read_vec_string(std::string file_name);
};

#endif