#include <iostream>
#include "base/gps.h"
#include "util/io.h"

// #include "base/similarity_transform.h"

using namespace colmap;
// const std::string WORK_SPACE_PATH = "@WORK_SPACE_PATH@";

int main(int argc, char** argv)
{
    
    // std::string gps_pose_path = "/home/yd/calibra_ws/bag/UAV_images+all/gps_pose.txt";
    // std::vector<GpsData> gps_datas;
    // read_gps_data(gps_datas, gps_pose_path.c_str());

    // std::cout << "gps size is: " << gps_datas.size() << std::endl;
    // Eigen::Vector3d centor(234109.49812860251,3078592.7338668993,0);
    // GPSTransform gps_trans_form;
    // for(const auto gps : gps_datas)
    // {
    //     auto utm = gps_trans_form.lla_to_utm(gps.lat, gps.lon, gps.alt);
    //     utm = utm - centor;
    //     std::cout << gps.frame_name << " utm is: " << utm << std::endl;
    // }
    // std::vector<Eigen::Vector3d> src;
    // std::vector<Eigen::Vector3d> dst;
    // SimilarityTransform3 tform;
    // tform.Estimate(src, dst);
    // Transform(tform);

    return 1;
}