#ifndef SRM_IC_2023_MODULES_COORDINATE_COORDINATE_H_
#define SRM_IC_2023_MODULES_COORDINATE_COORDINATE_H_

#include <Eigen/Core>
#include <opencv2/core/mat.hpp>
#include "common/syntactic-sugar.h"

namespace coordinate {
using Point2D = cv::Point2f;       ///< 2D 点坐标，变量名标记 p2d_ 前缀
using Point3D = cv::Point3d;       ///< 3D 点坐标，变量名标记 p3d_ 前缀
using RMat = Eigen::Matrix3d;      ///< 3x3 旋转矩阵，变量名标记 rm_ 前缀
using TMat = cv::Mat;              ///< 变换矩阵，变量名标记 tm_ 前缀
using ExtTMat = Eigen::Matrix4d;   ///< 4x4 旋转、位移变换矩阵，变量名标记 etm_ 前缀
using CTVec = Eigen::Vector3d;     ///< 3x1 直角坐标位移向量（以 (x, y, z) 表示，正方向依次为：右移、下移、前移）, 变量名标记 ctv_ 前缀
using STVec = Eigen::Vector3d;     ///< 3x1 球坐标位移向量（以 (phi, theta, r) 表示，正方向依次为：右偏、上仰）, 变量名标记 stv_ 前缀
using ExtCTVec = Eigen::Vector4d;  ///< 4x1 扩展直角坐标位移向量 (x, y, z, 1), 变量名标记 ectv_ 前缀
using EAngle = Eigen::Vector3d;    ///< 3x1 欧拉角（以 (roll, yaw, pitch) 表示，正方向依次为：右滚、右偏、上仰），变量名标记 ea_ 前缀

/// PnP 解算数据包
struct PnPInfo {
  CTVec ctv_cam;    ///< 目标中心点的相机坐标系直角坐标
  CTVec ctv_world;  ///< 目标中心点的世界坐标系直角坐标
  STVec stv_cam;    ///< 目标中心点的相机坐标系球坐标
  STVec stv_world;  ///< 目标中心点的世界坐标系球坐标
  RMat rm_cam;      ///< 目标自身相对相机的旋转矩阵
  EAngle ea_cam;    ///< 目标自身相对相机的欧拉角
};

/// 坐标系求解器类
class CoordSolver {
 private:
  TMat tm_intrinsic_;   ///< 相机内参
  TMat tm_distortion_;  ///< 相机外参
  ExtTMat etm_ic_;      ///< 陀螺仪坐标系转换到相机坐标系
  ExtTMat etm_ci_;      ///< 相机坐标系转换到陀螺仪坐标系
  CTVec ctv_iw_;        ///< 陀螺仪相对世界坐标系原点的位移
  CTVec ctv_cw_;        ///< 相机相对世界坐标系原点的位移

 public:
  /**
   * @brief 将旋转矩阵转换为欧拉角
   * @param [in] rm 旋转矩阵
   * @return 转换后的欧拉角
   */
  static EAngle RMatToEAngle(RMat REF_IN rm);

  /**
   * @brief 将欧拉角转换为旋转矩阵
   * @param [in] ea 欧拉角
   * @return 转换后的旋转矩阵
   */
  static RMat EAngleToRMat(EAngle REF_IN ea);

  /**
   * @brief 将直角坐标转换为球坐标
   * @param [in] ctv 直角坐标
   * @return 转换后的球坐标
   */
  static STVec CTVecToSTVec(CTVec REF_IN ctv);

  /**
   * @brief 将球坐标转换为直角坐标
   * @param [in] stv 球坐标
   * @return 转换后的直角坐标
   */
  static CTVec STVecToCTVec(STVec REF_IN stv);

  attr_reader_ref(ctv_iw_, CTVecIMUWorld)  ///< 陀螺仪相对世界坐标系原点的位移
  attr_reader_ref(ctv_cw_, CTVecCamWorld)  ///< 相机相对世界坐标系原点的位移

  /**
   * @brief 初始化坐标系参数
   * @param [in] config_file 配置文件名
   * @param tm_intrinsic 相机内参
   * @param tm_distortion 相机外参
   * @return 初始化过程是否正常完成
   */
  bool Initialize(std::string REF_IN config_file, TMat tm_intrinsic, TMat tm_distortion);

  /**
   * @brief 解算 PnP 数据
   * @param [in] p3d_world 参考世界坐标
   * @param [in] p2d_pic 图像点位
   * @param [in] rm_imu 当前姿态
   * @param [out] pnp_info 输出信息
   */
  void SolvePnP(std::array<Point3D, 4> REF_IN p3d_world,
                std::array<Point2D, 4> REF_IN p2d_pic,
                RMat REF_IN rm_imu,
                PnPInfo REF_OUT pnp_info) const;

  /**
   * @brief 将相机坐标系坐标转换为世界坐标系坐标
   * @param [in] ctv_cam 相机坐标系坐标
   * @param [in] rm_imu 当前云台姿态
   * @return 世界坐标系坐标
   */
  CTVec CamToWorld(CTVec REF_IN ctv_cam, RMat REF_IN rm_imu) const;

  /**
   * @brief 将世界坐标系坐标转换为相机坐标系坐标
   * @param [in] ctv_world 世界坐标系坐标
   * @param [in] rm_imu 当前云台姿态
   * @return 相机坐标系坐标
   */
  CTVec WorldToCam(CTVec REF_IN ctv_world, RMat REF_IN rm_imu) const;

  /**
   * @brief 将相机坐标系坐标投影到图像坐标系中
   * @param [in] ctv_cam 相机坐标系坐标
   * @return 图像坐标系点位
   */
  Point2D CamToPic(CTVec REF_IN ctv_cam) const;
};
}

#endif  // SRM_IC_2023_MODULES_COORDINATE_COORDINATE_H_
