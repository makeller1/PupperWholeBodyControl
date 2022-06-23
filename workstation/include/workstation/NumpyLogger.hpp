#ifndef PUPPER_NPYLOGGER_HH_
#define PUPPER_NPYLOGGER_HH_

#include <vector>
#include <string>
#include <unordered_map>
#include "cnpy.h" // for saving numpy files
#include "workstation/WBCTask.hpp"
#include "eigen3/Eigen/Dense"

// This logger stores and saves relevant test data as a numpy npz file
class NumpyLogger{
    public:
        NumpyLogger(); // Constructor
        void logScalars(std::vector<std::string> name_vector, std::vector<double> scalar_vector);
        void logVectorXd(std::string name, const Eigen::VectorXd& data_vector);
        void saveData(std::string output_name = "out");
    private:
        std::unordered_map<std::string, std::vector<double>> data_map_;
        std::string test_log_location_;
};

#endif