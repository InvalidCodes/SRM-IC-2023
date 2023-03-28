#include <glog/logging.h>
#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "coordinate.h"

#define USE_SIMD  ///< 使用 SIMD 加速数学计算，取消定义则使用标准数学计算

#ifdef USE_SIMD
#include "simd/simd.h"
#endif

coordinate::EAngle coordinate::CoordSolver::RMatToEAngle(RMat REF_IN rm) {
  constexpr double y_cos_threshold = 1e-6;
  double x, y, z;
#ifdef USE_SIMD
  auto y_cos = static_cast<double>(simd::sqrt_f(static_cast<float>(rm(0, 0) * rm(0, 0) + rm(1, 0) * rm(1, 0))));
  float atan2_y[4] = {-static_cast<float>(rm(1, 2)), -static_cast<float>(rm(2, 0)),
                      static_cast<float>(rm(2, 1)), static_cast<float>(rm(1, 0))},
      atan2_x[4] = {static_cast<float>(rm(1, 1)), static_cast<float>(y_cos),
                    static_cast<float>(rm(2, 2)), static_cast<float>(rm(0, 0))},
      atan2_result[4];
  simd::atan2_4f(atan2_y, atan2_x, atan2_result);
  if (y_cos < y_cos_threshold) {
    x = static_cast<double>(atan2_result[0]);
    y = static_cast<double>(atan2_result[1]);
    z = 0;
  } else {
    x = static_cast<double>(atan2_result[2]);
    y = static_cast<double>(atan2_result[1]);
    z = static_cast<double>(atan2_result[3]);
  }
#else
  double y_cos = sqrt(rm(0, 0) * rm(0, 0) + rm(1, 0) * rm(1, 0));
  if (y_cos < y_cos_threshold) {
    x = atan2(-rm(1, 2), rm(1, 1));
    y = atan2(-rm(2, 0), y_cos);
    z = 0;
  } else {
    x = atan2(rm(2, 1), rm(2, 2));
    y = atan2(-rm(2, 0), y_cos);
    z = atan2(rm(1, 0), rm(0, 0));
  }
#endif
  return {z, y, x};
}

coordinate::RMat coordinate::CoordSolver::EAngleToRMat(EAngle REF_IN ea) {
  RMat rm_z, rm_y, rm_x;
#ifdef USE_SIMD
  float ea_f[4] = {static_cast<float>(ea[0]), static_cast<float>(ea[1]), static_cast<float>(ea[2]), 0},
      ea_sin_f[4], ea_cos_f[4];
  simd::sin_cos_4f(ea_f, ea_sin_f, ea_cos_f);
  double ea_sin[3] = {static_cast<double>(ea_sin_f[0]),
                      static_cast<double>(ea_sin_f[1]),
                      static_cast<double>(ea_sin_f[2])},
      ea_cos[3] = {static_cast<double>(ea_cos_f[0]),
                   static_cast<double>(ea_cos_f[1]),
                   static_cast<double>(ea_cos_f[2])};
#else
  double ea_sin[3] = {sin(ea[0]), sin(ea[1]), sin(ea[2])},
      ea_cos[3] = {cos(ea[0]), cos(ea[1]), cos(ea[2])};
#endif
  rm_z << ea_cos[0], -ea_sin[0], 0,
      ea_sin[0], ea_cos[0], 0,
      0, 0, 1;
  rm_y << ea_cos[1], 0, ea_sin[1],
      0, 1, 0,
      -ea_sin[1], 0, ea_cos[1];
  rm_x << 1, 0, 0,
      0, ea_cos[2], -ea_sin[2],
      0, ea_sin[2], ea_cos[2];
  return rm_z * rm_y * rm_x;
}

coordinate::CTVec coordinate::CoordSolver::STVecToCTVec(STVec REF_IN stv) {
  CTVec ctv;
#ifdef USE_SIMD
  float stv_sin_cos[4] = {static_cast<float>(stv.y()), static_cast<float>(stv.x()), 0, 0}, stv_sin[4], stv_cos[4];
  simd::sin_cos_4f(stv_sin_cos, stv_sin, stv_cos);
  auto y_cos = static_cast<double>(stv_cos[0]), y_sin = static_cast<double>(stv_sin[0]),
      x_cos = static_cast<double>(stv_cos[1]), x_sin = static_cast<double>(stv_sin[1]);
  ctv[0] = stv.z() * y_cos * x_sin;
  ctv[1] = -stv.z() * y_sin;
  ctv[2] = stv.z() * y_cos * x_cos;
#else
  ctv[0] = stv.z() * cos(stv.y()) * sin(stv.x());
  ctv[1] = -stv.z() * sin(stv.y());
  ctv[2] = stv.z() * cos(stv.y()) * cos(stv.x());
#endif
  return ctv;
}

coordinate::STVec coordinate::CoordSolver::CTVecToSTVec(CTVec REF_IN ctv) {
  STVec stv;
#ifdef USE_SIMD
  float atan2_y[4] = {static_cast<float>(ctv.x()), static_cast<float>(-ctv.y()), 0, 0}, atan2_result[4], atan2_x[4] =
      {static_cast<float>(ctv.z()), simd::sqrt_f(static_cast<float>(ctv.x() * ctv.x() + ctv.z() * ctv.z())), 0, 0};
  simd::atan2_4f(atan2_y, atan2_x, atan2_result);
  stv[0] = static_cast<double>(atan2_result[0]);
  stv[1] = static_cast<double>(atan2_result[1]);
  stv[2] = static_cast<double>(simd::sqrt_f(
      static_cast<float>(ctv.x() * ctv.x() + ctv.y() * ctv.y() + ctv.z() * ctv.z())));
#else
  stv[0] = atan2(ctv.x(), ctv.z());
  stv[1] = atan2(-ctv.y(), sqrt(ctv.x() * ctv.x() + ctv.z() * ctv.z()));
  stv[2] = sqrt(ctv.x() * ctv.x() + ctv.y() * ctv.y() + ctv.z() * ctv.z());
#endif
  return stv;
}

bool coordinate::CoordSolver::Initialize(std::string REF_IN config_file, TMat tm_intrinsic, TMat tm_distortion) {
  cv::FileStorage coord_config;
  coord_config.open(config_file, cv::FileStorage::READ);
  if (!coord_config.isOpened()) {
    LOG(ERROR) << "Failed to open coordinate configuration file " << config_file << ".";
    return false;
  }
  std::vector<double> ea_cw_std, ctv_ci_std, ctv_iw_std;
  coord_config["EA_CAM_WORLD"] >> ea_cw_std;
  coord_config["CTV_CAM_IMU"] >> ctv_ci_std;
  coord_config["CTV_IMU_WORLD"] >> ctv_iw_std;
  if (ea_cw_std.size() != 3 || ctv_ci_std.size() != 3 || ctv_iw_std.size() != 3) {
    LOG(ERROR) << "Failed to read coordinate configurations.";
    return false;
  }
  ctv_iw_ << ctv_iw_std[0], ctv_iw_std[1], ctv_iw_std[2];
  ctv_cw_ << ctv_ci_std[0], ctv_ci_std[1], ctv_ci_std[2];
  ctv_cw_ += ctv_iw_;
  EAngle ea_cw;
  ea_cw << ea_cw_std[0], ea_cw_std[1], ea_cw_std[2];
  RMat rm_cw = EAngleToRMat(ea_cw);
  etm_ic_ << rm_cw(0, 0), rm_cw(0, 1), rm_cw(0, 2), ctv_ci_std[0],
      rm_cw(1, 0), rm_cw(1, 1), rm_cw(1, 2), ctv_ci_std[1],
      rm_cw(2, 0), rm_cw(2, 1), rm_cw(2, 2), ctv_ci_std[2],
      0, 0, 0, 1;
  bool invertible;
  double determinant;
  etm_ic_.computeInverseAndDetWithCheck(etm_ci_, determinant, invertible);
  if (!invertible) {
    ctv_cw_ = {};
    ctv_iw_ = {};
    etm_ic_ = {};
    etm_ci_ = {};
    LOG(ERROR) << "The extended IMU to Camera transformation matrix is not invertible. Please check your data.";
    return false;
  }
  tm_intrinsic_ = std::move(tm_intrinsic);
  tm_distortion_ = std::move(tm_distortion);
  LOG(INFO) << "Initialized coordinate solver.";
  return true;
}

void coordinate::CoordSolver::SolvePnP(
    std::array<Point3D, 4> REF_IN p3d_world,
    std::array<Point2D, 4> REF_IN p2d_pic,
    RMat REF_IN rm_imu,
    PnPInfo REF_OUT pnp_info) const {
  cv::Mat rv_cam_cv, ctv_cam_cv, rm_cam_cv;
  cv::solvePnP(p3d_world, p2d_pic, tm_intrinsic_, tm_distortion_, rv_cam_cv, ctv_cam_cv,
               false, cv::SOLVEPNP_AP3P);
  cv::Rodrigues(rv_cam_cv, rm_cam_cv);
  cv::cv2eigen(rm_cam_cv, pnp_info.rm_cam);
  cv::cv2eigen(ctv_cam_cv, pnp_info.ctv_cam);
  pnp_info.ea_cam = RMatToEAngle(pnp_info.rm_cam);
  pnp_info.ctv_world = CamToWorld(pnp_info.ctv_cam, rm_imu);
  pnp_info.stv_cam = CTVecToSTVec(pnp_info.ctv_cam);
  pnp_info.stv_world = CTVecToSTVec(pnp_info.ctv_world);
}

coordinate::CTVec coordinate::CoordSolver::CamToWorld(CTVec REF_IN ctv_cam, RMat REF_IN rm_imu) const {
  ExtCTVec ectv_cam, ectv_imu;
  CTVec ctv_imu, ctv_world;
  ectv_cam << ctv_cam[0], ctv_cam[1], ctv_cam[2], 1;
  ectv_imu = etm_ci_ * ectv_cam;
  ctv_imu << ectv_imu[0], ectv_imu[1], ectv_imu[2];
  ctv_imu -= ctv_iw_;
  return rm_imu * ctv_imu;
}

coordinate::CTVec coordinate::CoordSolver::WorldToCam(CTVec REF_IN ctv_world, RMat REF_IN rm_imu) const {
  ExtCTVec ectv_cam, ectv_imu;
  CTVec ctv_imu, ctv_cam;
  ctv_imu = rm_imu.transpose() * ctv_world;
  ctv_imu += ctv_iw_;
  ectv_imu << ctv_imu[0], ctv_imu[1], ctv_imu[2], 1;
  ectv_cam = etm_ic_ * ectv_imu;
  ctv_cam << ectv_cam[0], ectv_cam[1], ectv_cam[2];
  return ctv_cam;
}

coordinate::Point2D coordinate::CoordSolver::CamToPic(CTVec REF_IN ctv_cam) const {
  Eigen::Matrix3d tm_intrinsic_eigen;
  cv::cv2eigen(tm_intrinsic_, tm_intrinsic_eigen);
  CTVec result = (1.f / ctv_cam.z()) * tm_intrinsic_eigen * ctv_cam;
  return {static_cast<float>(result.x()), static_cast<float>(result.y())};
}
