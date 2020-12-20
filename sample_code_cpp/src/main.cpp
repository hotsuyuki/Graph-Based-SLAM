#include <iostream>
#include <string>

#include "cxxopts.hpp"
#include "FrameworkFactory.h"
#include "SlamLauncher.h"


int main(int argc, char* argv []) {
  // Decomposes argv[0] into directory path and file name
  std::string argv0(argv[0]);
  std::size_t last_slash_pos = argv0.find_last_of('/');
  std::string executable_directory = argv0.substr(0, last_slash_pos + 1);
  std::string executable_filename = argv0.substr(last_slash_pos + 1);

  std::string help_string = "Sample SLAM: Scan matching as a front-end, Graph based SLAM as a back-end";
  cxxopts::Options option_parser(executable_filename, help_string); 

  std::string sensor_data_filename;
  std::string viz_mode;

  // Parses program options
  // https://tadaoyamaoka.hatenablog.com/entry/2019/01/30/235251 (Japanese)
  try {
    option_parser.add_options()
      ("data-file", "String: Path to sensor data file (.lsc)", cxxopts::value<std::string>())
      ("viz-mode", "String: Visualization mode ... scan | odom | slam", cxxopts::value<std::string>())
      ("h,help", "Print usage");

    option_parser.parse_positional({"data-file", "viz-mode"});
    auto options = option_parser.parse(argc, argv);

    if (options.count("help")) {
      std::cout << option_parser.help({}) << std::endl;
      return EXIT_SUCCESS;
    }

    sensor_data_filename = options["data-file"].as<std::string>();
    viz_mode = options["viz-mode"].as<std::string>();

    std::cout << "\n";
    std::cout << "sensor_data_filename = " << sensor_data_filename << "\n";
    std::cout << "viz_mode = " << viz_mode << "\n";
    std::cout << "\n";
  }
  catch (cxxopts::OptionException &e) {
    std::cout << option_parser.usage() << "\n";
    std::cerr << e.what() << "\n";
    std::exit(EXIT_FAILURE);
  }

  std::string framework_config_filename = executable_directory + "../../framework.cfg";
  sample_slam::FrameworkFactory framework_factory;
  framework_factory.CustomizeFramework(framework_config_filename);
  sample_slam::SlamLauncher* slam_launcher_ptr = framework_factory.GetSlamLauncherPtr();

  if(!slam_launcher_ptr->OpenSensorDataFile(sensor_data_filename)) {
    return EXIT_FAILURE;
  }

  if (viz_mode == VIZ_MODE_SCAN) {
    slam_launcher_ptr->VisualizeScan();
  } else if (viz_mode == VIZ_MODE_ODOMETRY || viz_mode == VIZ_MODE_SLAM) {
    slam_launcher_ptr->Run(viz_mode);
  } else {
    std::cerr << "Error: invalid option... \n";
    std::cerr << "The command should be like: ./main /path/to/data "
              << VIZ_MODE_SCAN << " | " << VIZ_MODE_ODOMETRY << " | " << VIZ_MODE_SLAM << "\n\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}