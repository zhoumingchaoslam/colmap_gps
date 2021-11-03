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

#include "base/triangulation.h"

#include "base/essential_matrix.h"
#include "base/pose.h"

namespace colmap {
void getEquation(Eigen::Vector2d cam_point, Eigen::Matrix3d R, Eigen::Vector3d t, Eigen::Matrix<double,2,3>& A, Eigen::Matrix<double, 2, 1>& b)
{
  A(0, 0) = R(0, 0) - cam_point(0, 0) * R(2, 0);
  A(0, 1) = R(0, 1) - cam_point(0, 0) * R(2, 1);
  A(0, 2) = R(0, 2) - cam_point(0, 0) * R(2, 2);
  b(0, 0) = t[0] * A(0, 0) + t[1] * A(0, 1) + t[2] * A(0, 2);
  A(1, 0) = R(1, 0) - cam_point(1, 0) * R(2, 0);
  A(1, 1) = R(1, 1) - cam_point(1, 0) * R(2, 1);
  A(1, 2) = R(1, 2) - cam_point(1, 0) * R(2, 2);
  b(1, 0) = t[0] * A(1, 0) + t[1] * A(1, 1) + t[2] * A(1, 2);
}

bool weight_mutiTriangle(std::vector<Eigen::Vector2d> pts, Camera camera, std::vector<Eigen::Matrix3d> R, std::vector<Eigen::Vector3d> t, Eigen::MatrixXd weight, Eigen::Vector3d& P_ls)
{
  int pts_size = pts.size();
  const int matrix_row = pts_size * 2;
  if (pts_size >= 3)
  {
    int index = 0;
    Eigen::MatrixXd AA(matrix_row, 3);
    Eigen::MatrixXd bb(matrix_row, 1);
    for (const auto& pt : pts)
    {
      // Eigen::Vector2d campt = camera.ImageToWorld(pt);
      Eigen::Vector2d campt = pt;
      Eigen::Vector3d t1 = R[index].inverse() * (-t[index]);
      Eigen::Matrix<double, 2, 3> A;
      Eigen::Matrix<double, 2, 1> b;
      getEquation(campt, R[index], t1, A, b);
      AA.block<2,3>(index * 2, 0) = A;
      bb.block<2,1>(index * 2, 0) = b;
      index++;
    }
    P_ls = (AA.transpose() * weight * AA).inverse() * (AA.transpose() * weight * bb);
    return true;
  }else{
    std::cout << "tracker pixel point less 4,can not insection........  " << pts_size << std::endl;
    return false;
  }
}

bool mutiTriangle(std::vector<Eigen::Vector2d> pts, Camera camera, std::vector<Eigen::Matrix3d> R, std::vector<Eigen::Vector3d> t, Eigen::Vector3d& P_ls)
{
  int pts_size = pts.size();
  const int matrix_row = pts_size * 2;
  if (pts_size >= 3)
  {
    int index = 0;
    Eigen::MatrixXd AA(matrix_row, 3);
    Eigen::MatrixXd bb(matrix_row, 1);
    for (const auto& pt : pts)
    {
      // Eigen::Vector2d campt = camera.ImageToWorld(pt);
      Eigen::Vector2d campt = pt;
      Eigen::Vector3d t1 = R[index].inverse() * (-t[index]);
      Eigen::Matrix<double, 2, 3> A;
      Eigen::Matrix<double, 2, 1> b;
      getEquation(campt, R[index], t1, A, b);
      AA.block<2,3>(index * 2, 0) = A;
      bb.block<2,1>(index * 2, 0) = b;
      index++;
    }
    P_ls = (AA.transpose() * AA).inverse() * (AA.transpose() * bb);
    return true;
  }else{
    std::cout << "tracker pixel point less 4,can not insection........" << std::endl;
    return false;
  }
}

Eigen::Vector2d ProjectPoints(Eigen::Vector3d cam_xyz, Eigen::Matrix3d R, Eigen::Vector3d t, Camera camera)
{
  //to check
  Eigen::Vector3d xyz =  R * cam_xyz + t;
  Eigen::Vector2d uv = camera.WorldToImage(Eigen::Vector2d((xyz[0]/xyz[2]), (xyz[1]/xyz[2])));
  return uv;
}


Eigen::Vector3d IterationInsection(const std::vector<Eigen::Vector2d>& pts, const Camera& camera,const  std::vector<Eigen::Matrix3d>& R, const std::vector<Eigen::Vector3d>& t)
{
  // pts 是归一化坐标
  double k0 = 1.5;
  double k1 = 2.5;
  int pts_size = pts.size();
  // Eigen::MatrixXd weigth(pts_size*2, pts_size*2);
  // weigth.setIdentity();
  Eigen::Vector3d cam_xyz;
  if (!mutiTriangle(pts, camera, R, t, cam_xyz))
  {
    std::cout << "point size is less than 4 " << std::endl;
  }
  // cam_xyz = Eigen::Vector3d(-24.8, 17.44, 3.26);
  Eigen::Vector3d cam_xyz_pre = cam_xyz;
  std::vector<double> vec_delta;
  Eigen::VectorXd weight_tmp(pts_size,1);
  weight_tmp.setOnes();
  Eigen::Vector3d xyz_curr;

  // std::cout << "weight tmp is :" << weight_tmp << std::endl;
  while(true)
  {
    // Eigen::VectorXd d(pts_size);
    Eigen::Vector2d pix_uv;
    Eigen::VectorXd weight(pts_size,1);
    double delta = 0;

    for (int i = 0; i < pts_size; i++)
    {
      pix_uv = ProjectPoints(cam_xyz, R[i], t[i], camera);
      auto point_uv = camera.WorldToImage(pts[i]);
      double deltax = pix_uv[0] - point_uv[0];
      double deltay = pix_uv[1] - point_uv[1];
      vec_delta.push_back(std::sqrt(std::pow(deltax, 2) + std::pow(deltay, 2)));

      delta += std::sqrt(std::pow(deltax, 2) + std::pow(deltay, 2)) * weight_tmp[i];

    }
    delta = delta / (pts_size -2);
    //根据重投影误差计算权重
    for (int i = 0; i < pts_size; i++)
    {
      double error_uv = std::abs(vec_delta[i]);
      if (error_uv < k0 * delta)
      {
        weight[i] = 1;
      }else if (error_uv < k1 * delta && error_uv >= k0 * delta){
        weight[i] = delta / error_uv;
      }else if (error_uv >= k1 * delta){
        weight[i] = 0;
      }
    }
    weight_tmp = weight;
    // std::cout << "weight tmp is :" << weight_tmp << std::endl;

    Eigen::MatrixXd weight_matrix(pts_size * 2, pts_size * 2); //2 * pts_size
    weight_matrix.setIdentity();
    for(int i = 0; i < pts_size * 2; i++)
    {
      weight_matrix(i,i) = weight_tmp[int(i/2)];
    }
    std::cout << "weight_matrix is :" << weight_matrix << std::endl;
    if(!weight_mutiTriangle(pts,camera,R,t,weight_matrix,xyz_curr))
    {
      std::cout << "weight_mutiTriangle error !!!" << std::endl;
      break;
    }
    double error = (xyz_curr - cam_xyz_pre).norm();
    if (error < 0.01)
    {
      break;
    }else{
      cam_xyz = xyz_curr;
      cam_xyz_pre = xyz_curr;
    }
  }
  return xyz_curr;
}

Eigen::Vector3d TriangulateIDWMPoint(const TriangulationEstimator::PoseData& pose1,
                                     const TriangulationEstimator::PoseData& pose2,
                                     const Eigen::Vector2d& point1,
                                     const Eigen::Vector2d& point2)
{
  auto camera_triangulate = pose1.camera;
  Eigen::Matrix4d T1, T2;
  T1.setIdentity();
  T2.setIdentity();
  T1.block<3,4>(0,0) = pose1.proj_matrix;
  T2.block<3,4>(0,0) = pose2.proj_matrix;
  Eigen::Matrix4d T12 = (T1 * T2.inverse()).inverse();
  // std::cout << "T12 is " <<  std::endl << T12 << std::endl;
  Eigen::Matrix3d R12 = T12.block<3,3>(0,0);
  Eigen::Vector3d tt12 = T12.block<3,1>(0,3);
  //相机坐标系下的点坐标，只有xy，z设置为1
  // Eigen::Vector2d camera_point1 = camera_triangulate->ImageToWorld(point1);
  // Eigen::Vector2d camera_point2 = camera_triangulate->ImageToWorld(point2);
  Eigen::Vector3d f0 = point1.homogeneous();
  Eigen::Vector3d f1 = point2.homogeneous();
  Eigen::Vector3d Rf0 = R12 * f0;
  double p_norm = Rf0.cross(f1).norm();
  double q_norm = Rf0.cross(tt12).norm();
  double r_norm = f1.cross(tt12).norm();
  double weight = q_norm / (q_norm + r_norm);
  // weight = Eigen::Vector3d(q_norm[0] / weight[0], q_norm[1] / weight[1], q_norm[2] / weight[2]);
  double r_norm_p_norm = r_norm / p_norm;
  double q_norm_p_norm = q_norm / p_norm;
  

  Eigen::Vector3d point3d_cam = weight * (tt12 + (r_norm_p_norm) * (Rf0 + f1));

  Eigen::Vector3d point3d_world = (pose2.proj_matrix.transpose() * point3d_cam).head(3);
  Eigen::Vector3d lamda0 = r_norm_p_norm * Rf0;
  Eigen::Vector3d lamda1 = q_norm_p_norm * f1;

  double value1 = std::pow((tt12 + lamda0 - lamda1).norm(),2);
  double value2 = std::min(std::min(std::pow((tt12 + lamda0 + lamda1).norm(),2),std::pow((tt12 - lamda0 - lamda1).norm(),2)),
                           std::pow((tt12 - lamda0 + lamda1).norm(),2));
  if (value1 < value2)
  {
    return point3d_world;
  }else{
    return Eigen::Vector3d::Zero();
  }
}
Eigen::Vector3d TriangulatePoint(const Eigen::Matrix3x4d& proj_matrix1,
                                 const Eigen::Matrix3x4d& proj_matrix2,
                                 const Eigen::Vector2d& point1,
                                 const Eigen::Vector2d& point2) {
  Eigen::Matrix4d A;

  A.row(0) = point1(0) * proj_matrix1.row(2) - proj_matrix1.row(0);
  A.row(1) = point1(1) * proj_matrix1.row(2) - proj_matrix1.row(1);
  A.row(2) = point2(0) * proj_matrix2.row(2) - proj_matrix2.row(0);
  A.row(3) = point2(1) * proj_matrix2.row(2) - proj_matrix2.row(1);

  Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);

  return svd.matrixV().col(3).hnormalized();
}

std::vector<Eigen::Vector3d> TriangulatePoints(
    const Eigen::Matrix3x4d& proj_matrix1,
    const Eigen::Matrix3x4d& proj_matrix2,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2) {
  CHECK_EQ(points1.size(), points2.size());

  std::vector<Eigen::Vector3d> points3D(points1.size());

  for (size_t i = 0; i < points3D.size(); ++i) {
    points3D[i] =
        TriangulatePoint(proj_matrix1, proj_matrix2, points1[i], points2[i]);
  }

  return points3D;
}

Eigen::Vector3d TriangulateMultiViewPoint(
    const std::vector<Eigen::Matrix3x4d>& proj_matrices,
    const std::vector<Eigen::Vector2d>& points) {
  CHECK_EQ(proj_matrices.size(), points.size());

  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

  for (size_t i = 0; i < points.size(); i++) {
    const Eigen::Vector3d point = points[i].homogeneous().normalized();
    const Eigen::Matrix3x4d term =
        proj_matrices[i] - point * point.transpose() * proj_matrices[i];
    A += term.transpose() * term;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(A);

  return eigen_solver.eigenvectors().col(0).hnormalized();
}

Eigen::Vector3d TriangulateOptimalPoint(const Eigen::Matrix3x4d& proj_matrix1,
                                        const Eigen::Matrix3x4d& proj_matrix2,
                                        const Eigen::Vector2d& point1,
                                        const Eigen::Vector2d& point2) {
  const Eigen::Matrix3d E =
      EssentialMatrixFromAbsolutePoses(proj_matrix1, proj_matrix2);

  Eigen::Vector2d optimal_point1;
  Eigen::Vector2d optimal_point2;
  FindOptimalImageObservations(E, point1, point2, &optimal_point1,
                               &optimal_point2);

  return TriangulatePoint(proj_matrix1, proj_matrix2, optimal_point1,
                          optimal_point2);
}

std::vector<Eigen::Vector3d> TriangulateOptimalPoints(
    const Eigen::Matrix3x4d& proj_matrix1,
    const Eigen::Matrix3x4d& proj_matrix2,
    const std::vector<Eigen::Vector2d>& points1,
    const std::vector<Eigen::Vector2d>& points2) {
  std::vector<Eigen::Vector3d> points3D(points1.size());

  for (size_t i = 0; i < points3D.size(); ++i) {
    points3D[i] =
        TriangulatePoint(proj_matrix1, proj_matrix2, points1[i], points2[i]);
  }

  return points3D;
}

double CalculateTriangulationAngle(const Eigen::Vector3d& proj_center1,
                                   const Eigen::Vector3d& proj_center2,
                                   const Eigen::Vector3d& point3D) {
  const double baseline_length_squared =
      (proj_center1 - proj_center2).squaredNorm();

  const double ray_length_squared1 = (point3D - proj_center1).squaredNorm();
  const double ray_length_squared2 = (point3D - proj_center2).squaredNorm();

  // Angle between rays at point within the enclosing triangle,
  // see "law of cosines".
  const double angle = std::abs(std::acos(
      (ray_length_squared1 + ray_length_squared2 - baseline_length_squared) /
      (2.0 * std::sqrt(ray_length_squared1) * std::sqrt(ray_length_squared2))));

  if (IsNaN(angle)) {
    return 0;
  } else {
    // Triangulation is unstable for acute angles (far away points) and
    // obtuse angles (close points), so always compute the minimum angle
    // between the two intersecting rays.
    return std::min(angle, M_PI - angle);
  }
}

std::vector<double> CalculateTriangulationAngles(
    const Eigen::Vector3d& proj_center1, const Eigen::Vector3d& proj_center2,
    const std::vector<Eigen::Vector3d>& points3D) {
  // Baseline length between camera centers.
  const double baseline_length_squared =
      (proj_center1 - proj_center2).squaredNorm();

  std::vector<double> angles(points3D.size());

  for (size_t i = 0; i < points3D.size(); ++i) {
    // Ray lengths from cameras to point.
    const double ray_length_squared1 =
        (points3D[i] - proj_center1).squaredNorm();
    const double ray_length_squared2 =
        (points3D[i] - proj_center2).squaredNorm();

    // Angle between rays at point within the enclosing triangle,
    // see "law of cosines".
    const double angle = std::abs(std::acos(
        (ray_length_squared1 + ray_length_squared2 - baseline_length_squared) /
        (2.0 * std::sqrt(ray_length_squared1) *
         std::sqrt(ray_length_squared2))));

    if (IsNaN(angle)) {
      angles[i] = 0;
    } else {
      // Triangulation is unstable for acute angles (far away points) and
      // obtuse angles (close points), so always compute the minimum angle
      // between the two intersecting rays.
      angles[i] = std::min(angle, M_PI - angle);
    }
  }

  return angles;
}

}  // namespace colmap
