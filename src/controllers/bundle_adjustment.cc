// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch at inf.ethz.ch)

#include "controllers/bundle_adjustment.h"

#include <ceres/ceres.h>

#include "optim/bundle_adjustment.h"
#include "util/misc.h"

namespace colmap {
namespace {

// Callback functor called after each bundle adjustment iteration.
class BundleAdjustmentIterationCallback : public ceres::IterationCallback {
 public:
  explicit BundleAdjustmentIterationCallback(Thread* thread)
      : thread_(thread) {}

  virtual ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) {
    CHECK_NOTNULL(thread_);
    thread_->BlockIfPaused();
    if (thread_->IsStopped()) {
      return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    } else {
      return ceres::SOLVER_CONTINUE;
    }
  }

 private:
  Thread* thread_;
};

}  // namespace

BundleAdjustmentController::BundleAdjustmentController(
    const OptionManager& options, Reconstruction* reconstruction)
    : options_(options), reconstruction_(reconstruction) {}

void BundleAdjustmentController::Run() {
  CHECK_NOTNULL(reconstruction_);

  PrintHeading1("Global bundle adjustment   BundleAdjustmentController");

  const std::vector<image_t>& reg_image_ids = reconstruction_->RegImageIds();

  if (reg_image_ids.size() < 2) {
    std::cout << "ERROR: Need at least two views." << std::endl;
    return;
  }

  // Avoid degeneracies in bundle adjustment.
  reconstruction_->FilterObservationsWithNegativeDepth();

  BundleAdjustmentOptions ba_options = *options_.bundle_adjustment;
  ba_options.solver_options.minimizer_progress_to_stdout = true;

  BundleAdjustmentIterationCallback iteration_callback(this);
  ba_options.solver_options.callbacks.push_back(&iteration_callback);

  // Configure bundle adjustment.
  BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);
  }
  ba_config.SetConstantPose(reg_image_ids[0]);
  ba_config.SetConstantTvec(reg_image_ids[1], {0});

  //add gps
  // read gps pose
  // computer sim3 (sim_to_sentor)
  // std::vector<std::pair<Eigen::Vector3d, image_t>> gps_image_pair_ids;
  double pose_center_robust_fitting_error = 0.0;
  // bool add_gps_ = true;
  // if (add_gps_) {
  //   gps_datas_.clear();
  //   std::string gps_pose_path ="/home/yd/calibra_ws/bag/UAV_images+all/gps_pose.txt";
  //   std::vector<GpsData> org_gps_datas;
  //   if (read_gps_data(org_gps_datas, gps_pose_path.c_str())) {
  //     // Eigen::Vector3d centor(234109.49812860251,3078592.7338668993,0);
  //     GpsData first_position = org_gps_datas[0];
  //     GPSTransform gps_trans_form;
  //     Eigen::Vector3d centor = gps_trans_form.lla_to_utm(
  //         first_position.lat, first_position.lon, first_position.alt);
  //     for (const auto gps : org_gps_datas) {
  //       auto utm = gps_trans_form.lla_to_utm(gps.lat, gps.lon, gps.alt);
  //       utm = utm - centor;
  //       gps_datas_.push_back(utm);
  //     }
  //     std::vector<Eigen::Vector3d> src;
  //     std::vector<Eigen::Vector3d> dst;

  //     if (gps_datas_.size() > 3) {
  //       for (size_t i = 0; i < org_gps_datas.size(); i++) {
  //         const class Image* image =
  //             reconstruction_->FindImageWithName(org_gps_datas[i].frame_name);
  //         if (image->Tvec()[0] <= 0 && image->Tvec()[1] <= 0 &&
  //             image->Tvec()[2] <= 0) {
  //           continue;
  //         }

  //         src.push_back(image->Tvec());
  //         dst.push_back(gps_datas_[i]);
  //         gps_image_pair_ids.push_back(
  //             std::make_pair(gps_datas_[i], image->ImageId()));
  //       }
  //     }
  //     if (dst.size() > 3) {
  //       SimilarityTransform3 tform;
  //       tform.Estimate(src, dst);
  //       // std::cout << "sim3 is : " << std::endl << tform.Matrix() << std::endl;
  //       for (auto& ele : src) {
  //         tform.TransformPoint(&ele);
  //       }
  //       // std::cout << "src size is: " << src.size() << std::endl;
  //       // std::cout << "gps_datas_ size is: " << dst.size() << std::endl;
  //       if (src.size() > 3) {
  //         Eigen::VectorXd residual = (Eigen::Map<Eigen::Matrix<double, 3, -1>>(
  //                                         src[0].data(), 3, src.size()) -
  //                                     Eigen::Map<Eigen::Matrix<double, 3, -1>>(
  //                                         dst[0].data(), 3, dst.size()))
  //                                        .colwise()
  //                                        .norm();
  //         std::sort(residual.data(), residual.data() + residual.size());
  //         pose_center_robust_fitting_error = residual(residual.size() / 2);
  //         // 3d点和pose 转到 utm坐标系下
  //         reconstruction_->Transform(tform);
  //         std::cout << "finish trans sfm 3d and position !!! " << std::endl;
  //       } else {
  //         std::cout << "pose_center_robust_fitting_error can not computer !!!!!"
  //                   << std::endl;
  //       }
  //     } else {
  //       std::cout << "sim3 match size is small !!!" << std::endl;
  //     }
  //     // sfm pose 和 gps pose 计算误差权重
  //   } else {
  //     std::cout << "read gps pose wrong in file :" << gps_pose_path
  //               << std::endl;
  //   }
  //   ba_config.SetConstantGpsImageMapping(gps_image_pair_ids);
  // }

  // Run bundle adjustment.
  BundleAdjuster bundle_adjuster(ba_options, ba_config);
  bundle_adjuster.Solve(reconstruction_,pose_center_robust_fitting_error);

  // Normalize scene for numerical stability and
  // to avoid large scale changes in viewer.
  reconstruction_->Normalize();

  GetTimer().PrintMinutes();
}

}  // namespace colmap
