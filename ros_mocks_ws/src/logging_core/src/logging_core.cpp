// Copyright FARobot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "logging_core.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

static int fmtCallback(void * NotUsed, int argc, char ** argv, char ** azColName)
{
  int i;
  for (i = 0; i < argc; i++) {
    printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
  }
  printf("\n");
  return 0;
}

LoggingCore::~LoggingCore()
{
  sqlite3_close(syslogDB);
  std::cout << "System logger is uninitialized successfully" << std::endl;
}

LoggingCore::LoggingCore() : Node("logging_core")
{
  // ------ logging configuration ------
  std::string logging_folder = " ";

  this->declare_parameter<std::string>("logging_folder", " ");
  this->get_parameter("logging_folder", logging_folder);

  RCLCPP_INFO(get_logger(), "logging folder parameter: %s", logging_folder.c_str());

  std::string current_directory = "/home/farobot/far_app_data/app_log";
  RCLCPP_INFO(get_logger(), "directory path: %s", current_directory.c_str());

  if (logging_folder != " ") {
    current_directory = current_directory + "/" + logging_folder;
    std::filesystem::create_directories(current_directory);
  }

  // ====== database initialization ======
  // ------ system log db ------
  const char * logFile = "/home/farobot/far_app_data/app_log/test_log/systemlog.db";
  char * msgError;
  std::string sql =
    "CREATE TABLE IF NOT EXISTS syslog("
    "TIME        TEXT NOT NULL, "
    "event_type  TEXT NOT NULL, "
    "event_id    TEXT PRIMARY KEY NOT NULL, "
    "module_pub  INT  NOT NULL, "
    "module_sub  TEXT, "
    "level       TEXT, "
    "msg_cont    TEXT, "
    "robot_id    TEXT );";
  int rc = sqlite3_open(logFile, &this->syslogDB);
  int exit = sqlite3_exec(this->syslogDB, sql.c_str(), NULL, 0, &msgError);

  if (exit != SQLITE_OK) {
    std::cerr << "Error Create System Log Table" << std::endl;
    sqlite3_free(msgError);
  } else {
    std::cout << "System Log Table created Successfully" << std::endl;
  }

  // ------ fleet state db ------
  const char * fleetStateFile = "/home/farobot/far_app_data/app_log/test_log/fleetState.db";
  sql =
    "CREATE TABLE IF NOT EXISTS FleetState("
    "ts               TEXT NOT NULL, "
    "robot_id         TEXT PRIMARY KEY NOT NULL, "
    "robot_name       TEXT, "
    "model            TEXT, "
    "mode             INT, "
    "role             TEXT, "
    "pose_x           REAL, "
    "pose_y           REAL, "
    "pose_a           REAL, "
    "mileage          REAL, "
    "battery_percent  REAL, "
    "map              TEXT, "
    "fleet_name       TEXT, "
    "last_updated     TEXT )";
  rc = sqlite3_open(fleetStateFile, &this->fleetStateDB);
  // rc = sqlite3_open("file:memdb1?mode=memory&cache=shared", &this->fleetStateDB);
  sqlite3_exec(this->fleetStateDB, "PRAGMA journal_mode=WAL", NULL, 0, &msgError);
  sqlite3_exec(this->fleetStateDB, "DROP TABLE IF EXISTS FleetState;", NULL, 0, &msgError);
  exit = sqlite3_exec(this->fleetStateDB, sql.c_str(), NULL, 0, &msgError);

  if (exit != SQLITE_OK) {
    std::cerr << "Error Create Fleet State Table" << std::endl;
    sqlite3_free(msgError);
  } else {
    std::cout << "Fleet State Table created Successfully" << std::endl;
  }

  // ------ flow state db ------
  const char * flowStateFile = "/home/farobot/far_app_data/app_log/test_log/flowState.db";
  sql =
    "CREATE TABLE IF NOT EXISTS FlowState("
    "ts                 TEXT NOT NULL, "
    "flow_id            TEXT PRIMARY KEY NOT NULL, "
    "flow_name          TEXT, "
    "state              INT, "
    "progress           INT, "
    "start_time         REAL, "
    "end_time           REAL, "
    "robot_id           TEXT, "
    "fleet_name         TEXT, "
    "last_updated       TEXT NOT NULL) ";
  rc = sqlite3_open(flowStateFile, &this->flowStateDB);
  // rc = sqlite3_open("file:memdb2?mode=memory&cache=shared", &this->flowStateDB);
  sqlite3_exec(this->flowStateDB, "PRAGMA journal_mode=WAL", NULL, 0, &msgError);
  sqlite3_exec(this->flowStateDB, "DROP TABLE IF EXISTS FlowState;", NULL, 0, &msgError);
  exit = sqlite3_exec(this->flowStateDB, sql.c_str(), NULL, 0, &msgError);

  if (exit != SQLITE_OK) {
    std::cerr << "Error Create Flow State Table" << std::endl;
    sqlite3_free(msgError);
  } else {
    std::cout << "Flow State Table created Successfully" << std::endl;
  }

  // ====== subscription callback registration ======
  event_sub_ = this->create_subscription<Event>(
    "event", 10, std::bind(&LoggingCore::systemLogCallback, this, std::placeholders::_1));

  fleet_state_sub_ = this->create_subscription<FleetState>(
    "fleet_state", 10, std::bind(&LoggingCore::fleetStateCallback, this, std::placeholders::_1));

  flow_state_sub_ = this->create_subscription<FlowState>(
    "flow_state", 10, std::bind(&LoggingCore::flowStateCallback, this, std::placeholders::_1));

  // ====== logger mocker ======
  sys_logger_updater =
    this->create_wall_timer(0.05s, std::bind(&LoggingCore::sysLoggerUpdateCallback, this));

  // ====== agent mocker ======
  // agent_progress_updater =
  //   this->create_wall_timer(10s, std::bind(&LoggingCore::agentProgressUpdateCallback, this));

  // ====== flow mocker ======
  // flow_progress_updater =
  //   this->create_wall_timer(1s, std::bind(&LoggingCore::flowProgressUpdateCallback, this));
  // flow_fail_state_setter =
  //   this->create_wall_timer(180s, std::bind(&LoggingCore::flowFailStateCallback, this));
  // flow_new_tasks_creator =
  //   this->create_wall_timer(10s, std::bind(&LoggingCore::flowNewTasksCallback, this));
}

void LoggingCore::systemLogCallback(Event::UniquePtr msg)
{
  auto time_now = std::chrono::system_clock::now();
  std::time_t time_temp = std::chrono::system_clock::to_time_t(time_now);

  time = (std::ctime(&time_temp));
  time.pop_back();  // remove the \n

  try {
    std::string modulee = "";
    fmt = boost::format("%s;%s;%s;%s;%s;%s;%s") % time % msg->event_type % msg->event_id %
          msg->module % msg->level % msg->event_msg % msg->robot_id;
  } catch (...) {
    RCLCPP_INFO(this->get_logger(), "boost format failed");
  }

  std::string eventId = boost::uuids::to_string(this->generator());
  boost::erase_all(eventId, "-");
  RCLCPP_INFO(this->get_logger(), "Event ID: %s", eventId.c_str());

  // ------ sink system log to database ------
  std::string sql =
    "INSERT INTO syslog (TIME, event_type, event_id, module_pub, module_sub, level, msg_cont, "
    "robot_id) VALUES ('%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s');";

  // sql = boost::str(boost::format(sql)%time%msg->event_type%msg->event_id%msg->module%msg->module%msg->level%msg->event_msg%msg->robot_id);
  sql = boost::str(
    boost::format(sql) % time % msg->event_type % eventId % msg->module % msg->module % msg->level %
    msg->event_msg % msg->robot_id);

  /* Execute SQL statement */
  char * msgError;
  int rc = sqlite3_exec(this->syslogDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }
}

void LoggingCore::fleetStateCallback(FleetState::UniquePtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "--- fleet state messages ---");
  // --- Format the last-updated time ---
  auto time_now = std::chrono::system_clock::now();
  std::time_t time_temp = std::chrono::system_clock::to_time_t(time_now);
  char tt[100];
  time_t now = std::time(nullptr);
  auto tm_info = std::localtime(&now);
  std::strftime(tt, 100, "%Y-%m-%d %H:%M:%S", tm_info);
  puts(tt);

  for (auto robot : msg->robots) {
    // ------ sink flow state to database ------
    std::string sql =
      "INSERT OR REPLACE INTO FleetState (ts, robot_id, robot_name, model, mode, role, pose_x, "
      "pose_y, pose_a, mileage, battery_percent, map, fleet_name, last_updated) "
      "VALUES ('%s', '%s', '%s', '%s', '%s', '%s','%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s' "
      ");";

    sql = boost::str(
      boost::format(sql) % tt % robot.robot_id % robot.robot_name % robot.model % robot.mode %
      robot.role % robot.location.x % robot.location.y % robot.location.yaw % robot.mileage %
      robot.battery_percent % robot.map % msg->fleet_name % tt);

    /* Execute SQL statement */
    char * msgError;
    int rc = sqlite3_exec(this->fleetStateDB, sql.c_str(), fmtCallback, 0, &msgError);

    if (rc != SQLITE_OK) {
      fprintf(stderr, "SQL error: %s\n", msgError);
      sqlite3_free(msgError);
    } else {
      fprintf(stdout, "Records created successfully\n");
    }
  }
}

void LoggingCore::sysLoggerUpdateCallback()
{
  RCLCPP_INFO(this->get_logger(), "------ SYSTEM LOGGER UPDATE callback ------");

  // char tt[100];
  // time_t now = std::time(nullptr);
  // auto tm_info = std::localtime(&now);
  // std::strftime(tt, 100, "%Y-%m-%d %H:%M:%S", tm_info);
  // // --- update latest flow progress in rows ---
  // std::string strTime(tt);
  // // std::cout << strTime << std::endl;

  auto time_now = std::chrono::system_clock::now();
  std::time_t time_temp = std::chrono::system_clock::to_time_t(time_now);

  time = (std::ctime(&time_temp));
  time.pop_back();  // remove the \n

  std::string eventId = boost::uuids::to_string(this->generator());
  boost::erase_all(eventId, "-");
  RCLCPP_INFO(this->get_logger(), "Event ID: %s", eventId.c_str());

  std::string sql =
    "INSERT INTO syslog (TIME, event_type, event_id, module_pub, module_sub, level, msg_cont, "
    "robot_id) VALUES ('%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s');";                    

  sql = boost::str(
    boost::format(sql) % time % "logger_test" % eventId % "logging_core" % "logging_core" % "INFO" %
    "System log test" % "fb_x");

  /* Execute SQL statement */
  char * msgError;
  int rc = sqlite3_exec(this->syslogDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }
}

void LoggingCore::agentProgressUpdateCallback()
{
  RCLCPP_INFO(this->get_logger(), "------ AGENT PROGRESS UPDATE callback ------");

  char tt[100];
  time_t now = std::time(nullptr);
  auto tm_info = std::localtime(&now);
  std::strftime(tt, 100, "%Y-%m-%d %H:%M:%S", tm_info);
  // --- update latest flow progress in rows ---
  std::string strTime(tt);
  // std::cout << strTime << std::endl;

  std::string sql = "UPDATE FleetState SET battery_percent=battery_percent+1, ts='" + strTime +
                    "', last_updated='" + strTime + "' WHERE battery_percent<90 LIMIT 3";

  /* Execute SQL statement */
  char * msgError;
  int rc = sqlite3_exec(this->fleetStateDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }

  // --- update the flow state to complete when the progress is equal/larger than 100 ---
  sql = "UPDATE FleetState SET mode=0 WHERE battery_percent>=90";

  /* Execute SQL statement */
  rc = sqlite3_exec(this->fleetStateDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }
}

void LoggingCore::flowStateCallback(FlowState::UniquePtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "--- flow state messages ---");
  // --- Format the last-updated time ---
  auto time_now = std::chrono::system_clock::now();
  std::time_t time_temp = std::chrono::system_clock::to_time_t(time_now);
  char tt[100];
  time_t now = std::time(nullptr);
  auto tm_info = std::localtime(&now);
  std::strftime(tt, 100, "%Y-%m-%d %H:%M:%S", tm_info);
  puts(tt);

  for (auto flow : msg->flow_state) {
    // ------ sink flow state to database ------
    float scheTime = flow.scheduled_time.sec * 1e6 + flow.scheduled_time.nanosec * 1e-3;
    float startTime = flow.tasks[0].start_time.sec * 1e6 + flow.tasks[0].start_time.nanosec * 1e-3;
    float endTime = flow.tasks[0].end_time.sec * 1e6 + flow.tasks[0].end_time.nanosec * 1e-3;
    std::string sql =
      "INSERT OR REPLACE INTO FlowState (ts, flow_id, flow_name, state, progress, start_time, "
      "end_time, robot_id, fleet_name, last_updated) "
      "VALUES ('%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s', '%s');";

    sql = boost::str(
      boost::format(sql) % tt % flow.flow_id % flow.flow_name % flow.state % flow.complete_percent %
      startTime % endTime % flow.fleet_name % flow.fleet_name % tt);

    /* Execute SQL statement */
    char * msgError;
    int rc = sqlite3_exec(this->flowStateDB, sql.c_str(), fmtCallback, 0, &msgError);

    if (rc != SQLITE_OK) {
      fprintf(stderr, "SQL error: %s\n", msgError);
      sqlite3_free(msgError);
    } else {
      fprintf(stdout, "Records created successfully\n");
    }
  }
}

void LoggingCore::flowProgressUpdateCallback()
{
  RCLCPP_INFO(this->get_logger(), "------ PROGRESS UPDATE callback ------");

  char tt[100];
  time_t now = std::time(nullptr);
  auto tm_info = std::localtime(&now);
  std::strftime(tt, 100, "%Y-%m-%d %H:%M:%S", tm_info);
  // --- update latest flow progress in rows ---
  std::string strTime(tt);
  // std::cout << strTime << std::endl;

  std::string sql = "UPDATE FlowState SET state=1, progress=progress+1, last_updated='" + strTime +
                    "' WHERE state<2 LIMIT 3";

  /* Execute SQL statement */
  char * msgError;
  int rc = sqlite3_exec(this->flowStateDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }

  // --- update the flow state to complete when the progress is equal/larger than 100 ---
  sql = "UPDATE FlowState SET state=2 WHERE progress>=100";

  /* Execute SQL statement */
  rc = sqlite3_exec(this->flowStateDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }

  // --- update the flow state to mock data source from flow_state ---
  sql = "UPDATE FlowState SET last_updated='" + strTime + "'";

  /* Execute SQL statement */
  rc = sqlite3_exec(this->flowStateDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }

  // --- update the fleet state to mock data source from fleet_state ---
  sql = "UPDATE FleetState SET last_updated='" + strTime + "' LIMIT 2";

  /* Execute SQL statement */
  rc = sqlite3_exec(this->fleetStateDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }
}

void LoggingCore::flowFailStateCallback()
{
  RCLCPP_INFO(this->get_logger(), "------ FAIL STATE callback ------");
  // --- update flow state fail in rows ---
  std::string sql = "UPDATE FlowState SET state=3 WHERE state=1 ORDER BY ts LIMIT 1";

  /* Execute SQL statement */
  char * msgError;
  int rc = sqlite3_exec(this->flowStateDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }
}

void LoggingCore::flowNewTasksCallback()
{
  RCLCPP_INFO(this->get_logger(), "------ NEW TASKS callback ------");
  // --- add 2 new tasks ---
  auto time_now = std::chrono::system_clock::now();
  std::time_t time_temp = std::chrono::system_clock::to_time_t(time_now);

  char tt[100];
  time_t now = std::time(nullptr);
  auto tm_info = std::localtime(&now);
  std::strftime(tt, 100, "%Y-%m-%d %H:%M:%S", tm_info);
  puts(tt);

  // --- insert the 1st data row ---
  std::string flowId = boost::uuids::to_string(this->generator());
  boost::erase_all(flowId, "-");
  flowId = "FAR" + flowId;
  flowId = flowId.substr(0, 8);
  RCLCPP_INFO(this->get_logger(), "Flow ID: %s", flowId.c_str());

  std::string taskId = boost::uuids::to_string(this->generator());
  boost::erase_all(taskId, "-");
  taskId = "FAR" + taskId;
  taskId = taskId.substr(0, 8);

  // ------ sink flow state to database ------
  std::string sql =
    "INSERT INTO FlowState (ts, flow_id, flow_name, state, progress, start_time, end_time, robot_id, fleet_name, last_updated) "
    "VALUES ('%s', '%s', '%s', '%s', '%s', 'start_time', 'end_time', 'fb_0', '%s', '%s');";

  sql = boost::str(
    boost::format(sql) % tt % flowId % taskId % std::to_string(0).c_str() %
    std::to_string(0).c_str() % "AUO" % tt);

  /* Execute SQL statement */
  char * msgError;
  int rc = sqlite3_exec(this->flowStateDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }

  // --- insert the 2nd data row ---
  flowId = boost::uuids::to_string(this->generator());
  boost::erase_all(flowId, "-");
  flowId = "FAR" + flowId;
  flowId = flowId.substr(0, 8);
  RCLCPP_INFO(this->get_logger(), "Flow ID: %s", flowId.c_str());

  taskId = boost::uuids::to_string(this->generator());
  boost::erase_all(taskId, "-");
  taskId = "FAR" + taskId;
  taskId = taskId.substr(0, 8);

  // ------ sink flow state to database ------
  sql =
    "INSERT INTO FlowState (ts, flow_id, flow_name, state, progress, start_time, end_time, robot_id, fleet_name, last_updated) "
    "VALUES ('%s', '%s', '%s', '%s', '%s', 'start_time', 'end_time', 'robot_id', '%s', '%s');";

  sql = boost::str(
    boost::format(sql) % tt % flowId % taskId % std::to_string(0).c_str() %
    std::to_string(0).c_str() % "AUO" % tt);

  /* Execute SQL statement */
  rc = sqlite3_exec(this->flowStateDB, sql.c_str(), fmtCallback, 0, &msgError);

  if (rc != SQLITE_OK) {
    fprintf(stderr, "SQL error: %s\n", msgError);
    sqlite3_free(msgError);
  } else {
    fprintf(stdout, "Records created successfully\n");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto logging_pointer = std::make_shared<LoggingCore>();
  rclcpp::spin(logging_pointer);
  rclcpp::shutdown();
  return 0;
}