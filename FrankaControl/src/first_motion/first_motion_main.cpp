#include "shared/first_motion.h"

void writeLogToFile(const std::vector<franka::Record>& log);

int main() {
    std::string robot_ip = "192.168.40.45";
    
    first_motion::First_motion First_motion(robot_ip);
    controller::Controller Controller()

    return 0;
}

void writeLogToFile(const std::vector<franka::Record>& log) {
  if (log.empty()) {
    return;
  }
  try {
    Poco::Path temp_dir_path(Poco::Path::temp());
    temp_dir_path.pushDirectory("libfranka-logs");

    Poco::File temp_dir(temp_dir_path);
    temp_dir.createDirectories();

    std::string now_string =
        Poco::DateTimeFormatter::format(Poco::Timestamp{}, "%Y-%m-%d-%h-%m-%S-%i");
    std::string filename = std::string{"log-" + now_string + ".csv"};
    Poco::File log_file(Poco::Path(temp_dir_path, filename));
    if (!log_file.createFile()) {
      std::cout << "Failed to write log file." << std::endl;
      return;
    }
    std::ofstream log_stream(log_file.path().c_str());
    log_stream << franka::logToCSV(log);

    std::cout << "Log file written to: " << log_file.path() << std::endl;
  } catch (...) {
    std::cout << "Failed to write log file." << std::endl;
  }
}