#include <glog/logging.h>
#include "armor.h"

Armor::Armor(std::array<cv::Point2f, 4> REF_IN vertexes,
             coordinate::CoordSolver REF_IN coord_solver,
             coordinate::EAngle REF_IN euler_angle,
             ArmorSize size) : size_(size) {
  for (auto i = 0; i < 4; ++i) vertexes_[i] = vertexes[i];
  std::array<coordinate::Point3D, 4> p3d_world;
  switch (size_) {
    case ArmorSize::BIG: {
      p3d_world[0] = {-0.1125, 0.027, 0};
      p3d_world[1] = {-0.1125, -0.027, 0};
      p3d_world[2] = {0.1125, -0.027, 0};
      p3d_world[3] = {0.1125, 0.027, 0};
      break;
    }
    default: LOG(ERROR) << "Unknown armor size type. Fallback to small armor.";
    case ArmorSize::SMALL: {
      p3d_world[0] = {-0.066, 0.027, 0};
      p3d_world[1] = {-0.066, -0.027, 0};
      p3d_world[2] = {0.066, -0.027, 0};
      p3d_world[3] = {0.066, 0.027, 0};
      break;
    }
  }
  coord_solver.SolvePnP(p3d_world, vertexes_, coordinate::CoordSolver::EAngleToRMat(euler_angle), pnp_info_);
  center_ = coord_solver.CamToPic(pnp_info_.ctv_cam);
}
