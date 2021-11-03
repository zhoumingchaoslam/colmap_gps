#ifndef UTIL_IO_H
#define UTIL_IO_H
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

struct GpsData
{
    double lat;
    double lon;
    double alt;
    std::string frame_name;
};


template <typename T>
bool read_gps_data(std::vector<T>& gps_datas, const char* file_path)
{
    std::ifstream in(file_path, std::ios::out);
    if (!in.good())
    {
        return false;
    }
    std::string line;
    T gps_data;
    while(std::getline(in, line))
    {
        std::istringstream iss(line);
        if (!(iss >> gps_data.frame_name >> gps_data.lon >> gps_data.lat >> gps_data.alt))
        {
            break;
        }
        gps_datas.push_back(gps_data);
    }
    return true;
}




#endif