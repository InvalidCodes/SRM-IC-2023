#include <glog/logging.h>
#include "ballistic-solver.h"
#include "coordinate/coordinate.h"

void ballistic_solver::AirResistanceModel::SetParam(double c, double p, double t, double d, double m) {
  c_ = 0.5 * c * (1.293 * (p / 1013.25) * (273.15 / (273.15 + t))) * (0.25 * M_PI * d * d) / m;
}

ballistic_solver::CVec ballistic_solver::AirResistanceModel::operator()(double t, CVec REF_IN v) const {
  return -c_ * v.norm() * v;
}

void ballistic_solver::GravityModel::SetParam(double p) {
  double sin_p = sin(p * M_PI / 180), sin_2p = sin(p * M_PI / 90);
  g_ = 9.78 * (1 + 0.0052884 * sin_p * sin_p - 0.0000059 * sin_2p * sin_2p);
}

ballistic_solver::CVec ballistic_solver::GravityModel::operator()(double t, CVec REF_IN v) const {
  return {0, g_, 0};
}

void ballistic_solver::BallisticEquation::AddModel(std::shared_ptr<Model> REF_IN model) {
  models_.emplace_back(model);
}

ballistic_solver::CVec ballistic_solver::BallisticEquation::operator()(double t, CVec v, CVec acc) const {
  for (auto &&model : models_)
    acc += (*model)(t, v);
  return acc;
}

void ballistic_solver::BallisticSolver::AddModel(const std::shared_ptr<Model> &model) {
  solver_.f.AddModel(model);
}

void ballistic_solver::BallisticSolver::Initialize(CVec REF_IN intrinsic_x, double precision) {
  intrinsic_x_ = intrinsic_x;
  solver_.h = precision;
}

void ballistic_solver::BallisticSolver::SetParam(CVec REF_IN initial_v) {
  initial_v_ = initial_v;
  solver_.y = initial_v_ + intrinsic_v_;
  solver_.t = 0;
}

void ballistic_solver::BallisticSolver::Solve(
    std::function<bool(double t, CVec REF_IN v, CVec REF_IN x)> REF_IN solution_cond,
    std::function<bool(double t, CVec REF_IN v, CVec REF_IN x)> REF_IN iter_cond,
    std::vector<BallisticInfo> REF_OUT solutions) {
  solutions.clear();
  CVec current_x = intrinsic_x_ + solver_.y * solver_.h;
  for (size_t i = 0; iter_cond(solver_.t, solver_.y, current_x); ++i) {
    solver_.forward();
    current_x += solver_.y * solver_.h;
    if (solution_cond(solver_.t, solver_.y, current_x))
      solutions.push_back({solver_.t, coordinate::CoordSolver::CTVecToSTVec(initial_v_), solver_.y, current_x});
  }
}

bool ballistic_solver::BallisticSolver::Solve(
    CVec REF_IN target_x, double initial_v, CVec REF_IN intrinsic_v,
    BallisticInfo REF_OUT solution_out, double REF_OUT error_out) {
  constexpr size_t max_iter = 24;
  constexpr double error_limit = 0.005;
  intrinsic_v_ = intrinsic_v;
  bool exist_solution = false;
  double target_phi = target_x.x() / target_x.z();
  double min_theta = -M_PI / 2, max_theta = M_PI / 2, mid_theta;
  double min_phi = -M_PI / 2, max_phi = M_PI / 2, mid_phi;
  double min_error = Eigen::Vector2d(target_x.x(), target_x.z()).norm();
  BallisticInfo min_error_solution;
  size_t n = 0;
  double last_target_x_y;
  std::vector<BallisticInfo> solutions;
  std::function<bool(double t, CVec REF_IN v, CVec REF_IN x)> solution_cond = [&](
      double t, CVec REF_IN v, CVec REF_IN x) -> bool {
    bool approach_target = (target_x.y() - x.y()) * (target_x.y() - last_target_x_y) < 0;
    last_target_x_y = x.y();
    return approach_target;
  }, iter_cond = [&](double t, CVec REF_IN v, CVec REF_IN x) -> bool {
    return solutions.size() < 2 && x.y() < fmax(target_x.y(), intrinsic_x_.y());
  };
  while (n < max_iter && min_error > error_limit) {
    n += 1;
    mid_theta = (min_theta + max_theta) / 2;
    mid_phi = (min_phi + max_phi) / 2;
    last_target_x_y = target_x.y();
    SetParam(coordinate::CoordSolver::STVecToCTVec({mid_phi, mid_theta, initial_v}));
    Solve(solution_cond, iter_cond, solutions);
    if (!solutions.empty()) {
      exist_solution = true;
      BallisticInfo *solution;
      if (solutions.size() == 2) {
        if (Eigen::Vector2d(solutions[1].x.z(), solutions[1].x.x()).norm()
            < Eigen::Vector2d(target_x.z(), target_x.x()).norm()) {
          solution = &solutions[1];
          min_theta = mid_theta;
        } else if (Eigen::Vector2d(solutions[0].x.z(), solutions[0].x.x()).norm()
            > Eigen::Vector2d(target_x.z(), target_x.x()).norm()) {
          solution = &solutions[0];
          min_theta = mid_theta;
        } else {
          solution = &solutions[0];
          max_theta = mid_theta;
        }
      } else {
        if (Eigen::Vector2d(solutions[0].x.z(), solutions[0].x.x()).norm()
            < Eigen::Vector2d(target_x.z(), target_x.x()).norm())
          min_theta = mid_theta;
        else max_theta = mid_theta;
        solution = &solutions[0];
      }
      if (target_phi > solution->x.x() / solution->x.z())
        min_phi = mid_phi;
      else max_phi = mid_phi;
      double error = (target_x - solution->x).norm();
      if (error < min_error) {
        min_error = error;
        min_error_solution = *solution;
      }
    } else
      min_theta = mid_theta;
  }
  if (min_error / Eigen::Vector2d(target_x.z(), target_x.x()).norm() > 0.05)
    LOG(WARNING) << "Error of ballistic solution is more than 5%, the solution should not be trusted.";
  error_out = min_error;
  solution_out = min_error_solution;
  return exist_solution;
}
