#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <stdlib.h>
#include <robot_utilities/file_rw.hpp>

std::vector< std::vector<double> > file_rw::file_read_vec(std::string file_name)
{
    std::string tok;
    std::vector <std::vector <double> > vec;
    std::string line;
    std::string item;
    std::ifstream input_file;
    std::ifstream infile(file_name);
    if (infile.good())
    {
        input_file.open(file_name.c_str(),std::ios::in);
        while(!input_file.eof())
        {
            getline(input_file, line);
            if (!line.empty())
            {
                std::istringstream in(line);
                std::vector <double> row_vec;
                while (getline(in, item, ',')) 
                {
                    row_vec.push_back(atof(item.c_str()));
                }
                vec.push_back(row_vec);
            }
        }
        input_file.close();
    }
    else
    {
        std::cerr << "File/Path does not exist" << std::endl;
    }
    return vec;
} 

////////////////////////////////////////////////////////////

Eigen::MatrixXd file_rw::file_read_mat(std::string file_name)
{
    std::string tok;
    std::vector <std::vector <double> > vec;
    std::string line;
    std::string item;
    std::ifstream input_file;
    std::ifstream infile(file_name);
    if (infile.good())
    {
        input_file.open(file_name.c_str(),std::ios::in);
        while(!input_file.eof())
        {
            getline(input_file, line);
            if (!line.empty())
            {
                std::istringstream in(line);
                std::vector <double> row_vec;
                while (getline(in, item, ',')) 
                {
                    row_vec.push_back(atof(item.c_str()));
                }
                vec.push_back(row_vec);
            }
        }
        input_file.close();
    }
    else
    {
        std::cerr << "File/Path does not exist" << std::endl;
    }
    if (vec.size()!=0)
    {   
        Eigen::MatrixXd mat(vec.size(), vec[0].size());
        for (int i = 0; i < vec.size(); ++i)
        {
            mat.row(i) = Eigen::VectorXd::Map(&vec[i][0], vec[0].size());
        }
        return mat;
    }
    else
    {
        Eigen::MatrixXd mat(0,0);
        return mat;
    }
} 

////////////////////////////////////////////////////////////

void file_rw::file_write(std::string file_name, const std::vector< std::vector<int> >& vec)
{
    std::ofstream out_file;
    out_file.open(file_name.c_str(),std::ios::out);
    for (long i=0;i<vec.size();++i)
    {
        for (long j=0;j<vec[i].size();++j)
        {
            if (j!=vec[i].size()-1)
            {
                out_file << vec[i][j] << ",";    
            }
            else
            {
                out_file << vec[i][j] << std::endl;       
            }
        }
    }
    out_file.close();
}

////////////////////////////////////////////////////////////

void file_rw::file_write(std::string file_name, const std::vector< std::vector<float> >& vec)
{
    std::ofstream out_file;
    out_file.open(file_name.c_str(),std::ios::out);
    for (long i=0;i<vec.size();++i)
    {
        for (long j=0;j<vec[i].size();++j)
        {
            if (j!=vec[i].size()-1)
            {
                out_file << vec[i][j] << ",";    
            }
            else
            {
                out_file << vec[i][j] << std::endl;       
            }
        }
    }
    out_file.close();
}

////////////////////////////////////////////////////////////

void file_rw::file_write(std::string file_name, const std::vector< std::vector<double> >& vec)
{
    std::ofstream out_file;
    out_file.open(file_name.c_str(),std::ios::out);
    for (long i=0;i<vec.size();++i)
    {
        for (long j=0;j<vec[i].size();++j)
        {
            if (j!=vec[i].size()-1)
            {
                out_file << vec[i][j] << ",";    
            }
            else
            {
                out_file << vec[i][j] << std::endl;       
            }
        }
    }
    out_file.close();
}

////////////////////////////////////////////////////////////

void file_rw::file_write(std::string file_name, const Eigen::MatrixXd& mat)
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

////////////////////////////////////////////////////////////

void file_rw::file_write(std::string file_name, const Eigen::MatrixXi& mat)
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

////////////////////////////////////////////////////////////

void file_rw::file_write(std::string file_name, const Eigen::MatrixXf& mat)
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

////////////////////////////////////////////////////////////

void file_rw::file_write(std::string file_name, const std::vector< std::string >& vec)
{
    std::ofstream out_file;
    out_file.open(file_name.c_str(),std::ios::out);
    for (long i=0;i<vec.size();++i)
    {
        out_file << vec[i] << std::endl;
    }
    out_file.close();
}

////////////////////////////////////////////////////////////

std::vector<std::string> file_rw::file_read_vec_string(std::string file_name)
{
    std::string tok;
    std::vector <std::string> vec;
    std::string line;
    std::string item;
    std::ifstream input_file;
    std::ifstream infile(file_name);
    if (infile.good())
    {
        input_file.open(file_name.c_str(),std::ios::in);
        while(!input_file.eof())
        {
            getline(input_file, line);
            if (!line.empty())
            {
                std::istringstream in(line);
                while (getline(in, item, ','))
                {
                    vec.push_back(item);
                }
            }
        }
        input_file.close();
    }
    else
    {
        std::cerr << "File/Path does not exist" << std::endl;
    }
    return vec;
} 