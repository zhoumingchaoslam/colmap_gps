#include "../base/reconstruction.h"
#include "../base/database.h"
#include "../base/database_cache.h"

#include <opencv2/core/core.hpp> 
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/imgcodecs_c.h>

using namespace colmap;

int main(int argc, char** argv)
{
    // if (argc != 3)
    // {
    //     std::cout << "input error, ./localization *.txt(bin)path *.db" << std::endl;
    // }

    colmap::Reconstruction reconstruction_map,reconstruction;
    //读txt 或者 bin
    reconstruction_map.Read("/media/zmc/0053-2C75/jpg_down/sparse_BA_txt_txt_last");

    // Database database(argv[1]);
    Database database("/media/zmc/0053-2C75/jpg_down/match/match.db");
    DatabaseCache database_cache;
    size_t min_num_matches = 5;
    bool ignore_watermarks = false;
    std::set<std::string> image_names;
    image_names.emplace("1634022662.612247600_rear.jpg");
    image_names.emplace("1634022667.052247600_rear.jpg");
    database_cache.Load(database, min_num_matches,
                        ignore_watermarks,
                        image_names);

    
    reconstruction.Load(database_cache);
     // uint32_t image_id = 1;
  // for (const auto& image : database_cache.Images())
  // {
    uint32_t i = 1;
    int num = 0;
    auto images = database_cache.Images();
    auto image = images[1];
    auto image2 = images[2];
    std::vector<cv::KeyPoint> pts1, pts2;
    std::vector<cv::Point3f> pts3d;
    std::vector<cv::Point2f> pts2d;
    uint32_t size = image.NumPoints2D();
    // vector<cv::DMatch> matches;

    auto image_data = reconstruction_map.FindImageWithName("1634022662.612247600_rear.jpg");


    cv::KeyPoint pt1, pt2;
    for (; i < size; i++)
    {
      
      if(database_cache.CorrespondenceGraph().HasCorrespondences(1,i))
      {
        auto a = database_cache.CorrespondenceGraph().FindCorrespondences(1, i);
        std::cout << "corr is: image" << 1 << "  " << i << " match  image" << a[0].image_id << "  " << a[0].point2D_idx << std::endl;
        pt1.pt.x = image.Point2D(i).X();
        pt1.pt.y = image.Point2D(i).Y();
        pts1.push_back(pt1);

        pt2.pt.x = image2.Point2D(a[0].point2D_idx).X();
        pt2.pt.y = image2.Point2D(a[0].point2D_idx).Y();
        pts2.push_back(pt2);


        // if (image_data->IsPoint3DVisible(i))
        // {
        auto pt_2d = image_data->Point2D(i);
        //是否存在3d点
        if(reconstruction_map.ExistsPoint3D(pt_2d.Point3DId()))
        {
            Eigen::Vector3d pt_3d = reconstruction_map.Point3D(pt_2d.Point3DId()).XYZ();
            cv::Point3f cv_pt_3f = cv::Point3f(pt_3d[0], pt_3d[1], pt_3d[2]);
            pts3d.push_back(cv_pt_3f);
            pts2d.push_back(pt2.pt);
        }
        // }
        num++;
      }
    }
    std::cout <<"pts3d size is " << pts3d.size() << std::endl;
    cv::Mat r, t, inliers;
    cv::Mat K = (cv::Mat_<double>(3, 3) << 856.43, 0, 1440, 0, 856.43, 1440, 0, 0, 1);
    float change_score = 5.0;
    std::deque<std::pair<cv::Mat, cv::Mat>> pose_dq;

    bool computer_success = true;
    while(computer_success)
    {
        pose_dq.push_back(std::make_pair(r,t));
        change_score -= 0.2;
        computer_success = cv::solvePnPRansac(pts3d, pts2d, K, cv::Mat(), r, t, false, 300, change_score, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
    }
    r = pose_dq.back().first;
    t = pose_dq.back().second;

    // cv::solvePnP(pts3d, pts2d, K, cv::Mat(), r, t, false);
    cv::Mat R;
    cv::Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
    std::cout << "R=" << std::endl << R << std::endl;
    std::cout << "t=" << std::endl << t << std::endl;


    return 1;

    auto pic1 = cv::imread("/media/zmc/0053-2C75/jpg_down/match/1634022683.412247500_rear.jpg",CV_LOAD_IMAGE_COLOR);
    auto pic2 = cv::imread("/media/zmc/0053-2C75/jpg_down/match/1634022689.852247600_rear.jpg",CV_LOAD_IMAGE_COLOR);
    for (size_t i = 0; i < pts2.size(); i++)
    {
      cv::Point2f point1 = pts1[i].pt;
      cv::Point2f point2 = pts2[i].pt;
      cv::putText(pic1, std::to_string(i), point1, cv::FONT_HERSHEY_SIMPLEX,0.45, CV_RGB(255,230,0),1.8);
      cv::putText(pic2, std::to_string(i), point2, cv::FONT_HERSHEY_SIMPLEX,0.45, CV_RGB(255,230,0),1.8);
    }
    cv::imwrite("image1.jpg", pic1);
    cv::imwrite("image2.jpg", pic2);
    cv::waitKey(0);
    // image_id++;
    std::cout << "match size is " << num << std::endl;
  // }
    


    return 1;
}