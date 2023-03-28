#include <csignal>
#include <glog/logging.h>
#include "cli-arg-parser/cli-arg-parser.h"
#include "controller-base/controller-base.h"

std::atomic_bool controller::Controller::exit_signal_ = false;

void SignalHandler(int) {
  LOG(WARNING) << "Caught interrupt signal. Attempting to exit...";
  controller::Controller::exit_signal_ = true;
}

int main(int argc, char **argv) {
  cli_argv.Parse(argc, argv);
  std::unique_ptr<controller::Controller> controller;
  controller.reset(controller::CreateController(cli_argv.ControllerType()));
  if (!controller) return -1;
  if (!controller->Initialize()) return 1;
  std::signal(SIGINT, &SignalHandler);
  int ret = controller->Run();
  google::ShutdownGoogleLogging();
  return ret;
}
