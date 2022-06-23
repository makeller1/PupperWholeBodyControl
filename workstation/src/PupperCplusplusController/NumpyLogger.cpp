#include "workstation/NumpyLogger.hpp"
#include <iostream>
#include <chrono>
using std::string;
using std::vector;

namespace{
    // start timer (ms)
    double tic(int mode=0) {
        static std::chrono::_V2::system_clock::time_point t_start;
        
        if (mode==0)
            t_start = std::chrono::high_resolution_clock::now();
        else {
            auto t_end = std::chrono::high_resolution_clock::now();
            double t_delta = (t_end-t_start).count()*1E-6;
            return t_delta;
        }
        return 0.0;
    }
    // return elapsed time (ms)
    double toc() { return tic(1); }
}

NumpyLogger::NumpyLogger(){
    test_log_location_ = "/home/mathew/WBC_catkin_ws/src/PupperWholeBodyControl/workstation/src/PythonComms/test_logs/";
}

// Log multiple scalars with different names simultaneously
// inputs:
//  name_vector - names of each scalar
//  scalar_vector - vector containing the scalars with the same order as name_vector 
void NumpyLogger::logScalars(vector<std::string> name_vector, vector<double> scalar_vector){
    static double logging_ms = 0.0;
    tic();
    for (int i = 0; i < name_vector.size(); i++){
        string& name = name_vector.at(i);
        double& data = scalar_vector.at(i);
        data_map_[name].push_back(data);
    }
    // time and log the logging process
    logging_ms = logging_ms + toc();
    if (name_vector.at(0) == "time_ms"){
        NumpyLogger::logScalars({"log_ms"},{logging_ms});
        logging_ms = 0.0;
    }
}

// Log a vectorXd time series
void NumpyLogger::logVectorXd(std::string name, const Eigen::VectorXd& data_vector){
    vector<double> temp_vector(data_vector.size());
    std::copy(data_vector.data(), data_vector.data() + data_vector.size(), temp_vector.data());
    for (int i = 0; i < temp_vector.size(); i++){
        NumpyLogger::logScalars({name + "_" + std::to_string(i+1)},{temp_vector.at(i)});
    }
}

void NumpyLogger::saveData(string save_name){
    
    for (const auto& item : data_map_){
        const string& name = item.first;
        const vector<double>& data_vector = item.second;
        if (&item == &(*data_map_.begin())){
            cnpy::npz_save(test_log_location_+save_name+".npz", name, &data_vector[0], {data_vector.size()},"w");
        }
        else{
            cnpy::npz_save(test_log_location_+save_name+".npz", name, &data_vector[0], {data_vector.size()},"a");
        }
    }
}