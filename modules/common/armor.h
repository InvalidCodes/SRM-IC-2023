#ifndef SRM_IC_2023_MODULES_COMMON_ARMOR_H_
#define SRM_IC_2023_MODULES_COMMON_ARMOR_H_

#include <opencv2/core/types.hpp>
#include "syntactic-sugar.h"
#include "coordinate/coordinate.h"

/// 装甲板数据类
class Armor final {
 public:
  enum class ArmorSize { BIG, SMALL };  ///< 装甲板类型

  /**
   * @brief 构造装甲板对象
   * @param [in] vertexes 四个图像坐标点
   * @param [in] coord_solver 坐标系求解方式
   * @param [in] euler_angle 当前云台姿态欧拉角
   * @param size 装甲板类型
   */
  Armor(std::array<cv::Point2f, 4> REF_IN vertexes,
        coordinate::CoordSolver REF_IN coord_solver,
        coordinate::EAngle REF_IN euler_angle,
        ArmorSize size);

  /// 图像上的四个角点
  attr_reader_ref(vertexes_, Vertexes)
  /// 装甲板中心在图像上的位置
  attr_reader_ref(center_, Center)
  /// 装甲板中心在相机坐标系中的直角坐标
  attr_reader_ref(pnp_info_.ctv_cam, CTVecCam)
  /// 装甲板中心在相机坐标系中的球坐标
  attr_reader_ref(pnp_info_.stv_cam, STVecCam)
  /// 装甲板中心在世界坐标系中的直角坐标
  attr_reader_ref(pnp_info_.ctv_world, CTVecWorld)
  /// 装甲板中心在世界坐标系中的球坐标
  attr_reader_ref(pnp_info_.stv_world, STVecWorld)
  /// 装甲板自身相对于相机的旋转方向
  attr_reader_ref(pnp_info_.ea_cam, EAngleCam)
  /// 装甲板类型
  attr_reader_val(size_, Size)

 private:
  std::array<cv::Point2f, 4> vertexes_;  ///< 图像上的四个角点
  coordinate::PnPInfo pnp_info_;        ///< PnP 求解信息，记录当前装甲板的相机坐标和世界坐标
  cv::Point2f center_;                  ///< 装甲板中心在图像上的位置
  ArmorSize size_;                      ///< 装甲板类型
};

#endif  // SRM_IC_2023_MODULES_COMMON_ARMOR_H_
