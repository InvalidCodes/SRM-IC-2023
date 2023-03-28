#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include "common/armor.h"
#include "common/bbox_t.h"
#include "cli-arg-parser/cli-arg-parser.h"
#include "ballistic-solver/ballistic-solver.h"
#include "controller-sentry.h"

controller::Registry<controller::hero::HeroController> controller::hero::HeroController::registry_("sentry");

bool controller::hero::HeroController::Initialize() {
  bool ret = controller::Controller::Initialize("sentry");
  if (ret) LOG(INFO) << "Initialized sentry controller.";
  else LOG(INFO) << "Failed to initialize!";
  return ret;
}

int controller::hero::HeroController::Run() {
  double fps = 0, show_fps = 0;
  bool pause = false, show_warning = true;

  struct timespec ts_start{};
  coordinate::EAngle current_attitude{0, 0, 0};
  ballistic_solver::BallisticSolver ballistic_solver;
  auto ar_model = std::make_shared<ballistic_solver::AirResistanceModel>();
  ar_model->SetParam(0.26, 1002, 25, 0.0425, 0.041);
  ballistic_solver.AddModel(ar_model);
  auto g_model = std::make_shared<ballistic_solver::GravityModel>();
  g_model->SetParam(31);
  ballistic_solver.AddModel(g_model);
#if NDEBUG
  ballistic_solver.Initialize(coord_solver_.CTVecCamWorld(), 0.001);
#else
  ballistic_solver.Initialize(coord_solver_.CTVecCamWorld(), 0.01);
#endif

  constexpr auto frame_time_str = [](auto time_stamp) {
    static auto start_time = time_stamp;
    auto delta_time_ns = time_stamp - start_time;
    double delta_time_s = static_cast<double>(delta_time_ns) * 1e-9;
    auto delta_time_m = static_cast<uint32_t>(std::floor(delta_time_s / 60));
    delta_time_s -= delta_time_m * 60;
    auto delta_time_h = delta_time_m / 60;
    delta_time_m -= delta_time_h * 60;
    std::ostringstream ss_time;
    if (delta_time_h) ss_time << std::setw(2) << std::setfill('0') << delta_time_h << ":";
    ss_time << std::setw(2) << std::setfill('0') << delta_time_m << ":"
            << std::setw(5) << std::setfill('0') << std::fixed << std::setprecision(2)
            << delta_time_s;
    return ss_time.str();
  };

  auto update_frame_data = [&]() {
    if (pause) return false;
    auto ret = video_source_->GetFrame(frame_);
    if (ret) current_attitude = {frame_.receive_packet.roll, frame_.receive_packet.yaw, frame_.receive_packet.pitch};
    if (!ret && show_warning)
      LOG(WARNING) << "Failed to get frame data from video source."
                   << "Wait for reconnecting the camera or press Ctrl-C to exit.";
    show_warning = ret;
    return ret;
  };

  auto update_window = [&](std::string REF_IN title) {
    static uint32_t rec_frame_count = 0;
    std::ostringstream ss_fps;
    ss_fps << std::fixed << std::setprecision(0) << show_fps;
    if (!pause && show_warning) {
      if (cli_argv.Record()) {
        cv::Mat image = frame_.image.clone();
        video_writer_.Write(std::move(image));
        ++rec_frame_count;
      }
      // 在UI里放入图像，时间戳
      if (cli_argv.UI()) {
        cv::putText(frame_.image, frame_time_str(frame_.time_stamp),
                    cv::Point(0, 24), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 192, 0));
        cv::putText(frame_.image, "FPS: " + ss_fps.str(),
                    cv::Point(0, 48), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 192, 0));
        if (cli_argv.Record())
          cv::putText(frame_.image, "REC: " + std::to_string(rec_frame_count), cv::Point(0, 72),
                      cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 192));
        cv::imshow(title, frame_.image);
      }
    }
  };

  auto start_count_fps = [&]() {
    clock_gettime(CLOCK_REALTIME, &ts_start);
  };

  auto stop_count_fps = [&]() {
    struct timespec ts_end{};
    clock_gettime(CLOCK_REALTIME, &ts_end);
    auto delta_time_ns = ts_end.tv_nsec - ts_start.tv_nsec;
    if (delta_time_ns > 0)
      fps = 1e9 / static_cast<double>(delta_time_ns);
  };

  // q 退出 p 继续
  auto check_key = [&]() {
    if (cli_argv.UI()) {
      auto key = cv::waitKey(1);
      if (key == 'q') {
        LOG(INFO) << "CONTROL MSG: QUIT";
        exit_signal_ = true;
      } else if (key == 'p') {
        LOG(INFO) << "CONTROL MSG: " << (pause ? "RESUME" : "PAUSE");
        pause = !pause;
      }
    }
  };

  std::thread auto_log_fps([&]() {
    while (!exit_signal_) {
      if (!pause && show_warning) {
        show_fps = fps;
        LOG(INFO) << "FPS: " << fps;
      }
      sleep(1);
    }
  });

  auto fix_aim_point = [&](Armor REF_IN armor, ballistic_solver::CVec REF_IN intrinsic_v)
      -> ballistic_solver::CVec {
    ballistic_solver::BallisticInfo solution;
    double error;
    if (ballistic_solver.Solve(armor.CTVecWorld(), frame_.receive_packet.bullet_speed, intrinsic_v, solution, error)) {
      auto target_pic = coord_solver_.CamToPic(coord_solver_.WorldToCam(
          solution.x, coordinate::CoordSolver::EAngleToRMat(current_attitude)));
      cv::circle(frame_.image, target_pic, 2, cv::Scalar(192, 0, 192), 2);
      auto v_0_pic = coord_solver_.CamToPic(coord_solver_.WorldToCam(
          coordinate::CoordSolver::STVecToCTVec(solution.v_0),
          coordinate::CoordSolver::EAngleToRMat(current_attitude)));
      cv::circle(frame_.image, v_0_pic, 2, cv::Scalar(0, 0, 192), 2);
      return solution.v_0;
    } else return {0, 0, 0};
  };

  auto bbox_to_armor = [&](Armor::ArmorSize size = Armor::ArmorSize kAuto) {
    armors_.clear();
    for (size_t i = 0; i < 4; ++i)
      armors_ = armor_vertexes[i];
    for (const auto &vertexes : armor_vertexes)
      vertexes.emplace_back(vertexes, coord_solver_, current_attitude, size, &armor_center);
  };

  auto draw_outpost = [&](Armor REF_IN armor) {
    for (size_t i = 0; i < 4; ++i)
      cv::circle(frame_.image, coord_solver_.CamToPic(armor.CTVecCam()), 2, cv::Scalar(0, 192, 0), 2);

  }

  auto draw_armor = [&](Armor REF_IN armor) {
    for (size_t i = 0; i < 4; ++i)
      cv::line(frame_.image, armor.Vertexes()[i], armor.Vertexes()[(i + 1) % 4], cv::Scalar(0, 192, 0), 2);
    //cv::circle(frame_.image, coord_solver_.CamToPic(armor.CTVecCam()), 2, cv::Scalar(0, 192, 0), 2);
  };

  std::function<void(void *, Frame &)> patch_default_bullet_speed = [](void *, Frame &frame) -> void {
    frame.receive_packet.bullet_speed = 14;
  };
  if (!cli_argv.Serial())
    video_source_->RegisterFrameCallback(&patch_default_bullet_speed, this);

  cv::Point2f armor_center{45, 40};
  cv::MouseCallback on_mouse = [](int event, int x, int y, int flags, void *userdata) -> void {
    static bool armor_locked = false;
    switch (event) {
      case cv::EVENT_MOUSEMOVE:
        if (!armor_locked) {
          auto center = (cv::Point2f *) userdata;
          center->x = static_cast<float>(x);
          center->y = static_cast<float>(y);
        }
        break;
      case cv::EVENT_LBUTTONDOWN: armor_locked = true;
        break;
      case cv::EVENT_RBUTTONDOWN: armor_locked = false;
        break;
      default: break;
    }
  };

  if (cli_argv.UI()) {
    cv::namedWindow("SENTRY");
    cv::setMouseCallback("SENTRY", on_mouse, &armor_center);
  }

  while (!exit_signal_) {
    start_count_fps();
    update_frame_data();

    if (cli_argv.UI() && !pause && show_warning) {
      // boxes_
      std::array<cv::Point2f, 4> armor_vertexes = armor_detector(frame_.image); 
     //std::array<cv::Point2f, 4> armor_vertexes = {cv::Point2f{-45, -40}, {45, -40}, {45, 40}, {-45, 40}};
      //battlefield_ = BattleField()
      bbox_to_armor();
      for (auto &&p : armor_vertexes) p += armor_center;
      Armor armor{armor_vertexes, coord_solver_, current_attitude, Armor::ArmorSize::SMALL};
      draw_armor(armor);
      // 弹道补偿为0
      fix_aim_point(armor, {0, 0, 0});
    }

    update_window("SENTRY");
    stop_count_fps();
    check_key();
  }

  if (!cli_argv.Serial())
    video_source_->UnregisterFrameCallback(&patch_default_bullet_speed);

  LOG(INFO) << "Main loop finished. Waiting for background tasks to exit.";
  if (auto_log_fps.joinable()) auto_log_fps.join();
  cv::destroyAllWindows();
  return 0;
}
