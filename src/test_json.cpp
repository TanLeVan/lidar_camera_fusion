#include <fstream>
#include<nlohmann/json.hpp>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
void read_camera_martix(){
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("lidar_camera_fusion");
    std::string filePath = package_share_directory.append("/config/camera_matrix.json");
    std::ifstream f(filePath);
    nlohmann::json data;
    f>>data;
    std::cout << data["camera_matrix"][0][1] << std::endl;
}
int main(){
    read_camera_martix();
    return 0;
}