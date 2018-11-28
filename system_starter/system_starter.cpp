#include "system_starter.h"
#include "process.h"
#include "pugixml/pugixml.hpp"
#include "struct_defines.h"
#include <iostream>
#include <ostream>
#include <thread>

// use simple websocket server
#include "simple_websocket/server_ws.hpp"
using WsServer = SimpleWeb::SocketServer<SimpleWeb::WS>;

using namespace Bgs;
using namespace std;

GlobalShm *g_ptr_shm = NULL;
system_starter::Status starter_status;
system_starter::Config starter_config;
volatile sig_atomic_t flag_ctrl_handler = 1;


int32_t carto_pid = 0;
int32_t driver_pid = 0;
int32_t controller_pid = 0;

system_starter::SystemState current_system_state = system_starter::kIdle;
system_starter::SystemState target_system_state = system_starter::kIdle;
system_starter::ProcessRunConfig current_run_config;

bool run_heart_beat = true;

int32_t server_handle_msg(const std::string &message,
                          std::shared_ptr<WsServer::Connection> &connection);

int main(int argc, char **argv)
{
  // 用文件锁来实现“保证程序只运行一个实例”的功能
  int32_t lock_fd = 0;
  if (!common::is_single_process("starter.lck", lock_fd)) {
    PRINT_ERROR("there is a starter process already.");
    return -1;
  }
  // step1 read configs
  if (read_configs(argc, argv, starter_config) == BGS_ERROR
      || init_singals() == BGS_ERROR)
    return -1;

  // step2 create and initialise the shared memory and semophare
  if (create_and_init_shm() == BGS_ERROR) {
    PRINT_ERROR("shm init error.");
    exit(-1);
  }

// step 2.0 initialise a websocket server
#if 0
	uWS::Hub h;
	h.onConnection([](uWS::WebSocket<uWS::SERVER> * ws, HttpRequest req )
	{
		PRINT_INFO("connected...");
	} );
	h.onDisconnection([](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length)
	{
		PRINT_INFO("disconnected...");
	});
	h.onMessage([&h](uWS::WebSocket<uWS::SERVER> *ws, char *message, size_t length, uWS::OpCode opCode) 
	{
		if(opCode == uWS::BINARY)
		{
			
		}
		else if (opCode == uWS::TEXT)
			server_handle_msg( std::string(message, length) );
	});
	
	std::thread websocket_server_thread(
		[&h]()
		{
			if( h.listen(starter_config.server_port) )
			{
				PRINT_INFO_FMT("websocket start listening port: %d", starter_config.server_port);
				h.run();
			}
			
			PRINT_DEBUG("server thread exit.");
		}
	);
#else
  WsServer server;
  server.config.port = 3000;
  auto &echo = server.endpoint["^/?$"];

  echo.on_message = [](shared_ptr<WsServer::Connection> connection,
                       shared_ptr<WsServer::Message> message) {
    auto message_str = message->string();
    server_handle_msg(message_str, connection);
  };

  echo.on_open = [](shared_ptr<WsServer::Connection> connection) {
    PRINT_INFO("connected...");
  };

  // See RFC 6455 7.4.1. for status codes
  echo.on_close = [](shared_ptr<WsServer::Connection> connection, int status,
                     const string & /*reason*/) {
    PRINT_INFO("disconnected...");
  };

  // See
  // http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html,
  // Error Codes for error code meanings
  echo.on_error = [](shared_ptr<WsServer::Connection> connection,
                     const SimpleWeb::error_code &ec) {
    std::ostringstream os;
    os << "Server: Error in connection " << connection.get() << ". "
       << "Error: " << ec << ", error message: " << ec.message();
    PRINT_ERROR(os.str());
  };

  std::thread websocket_server_thread([&server]() { server.start(); });

#endif

  // step2.1 run driver
  if (current_run_config.run_driver) {
    if (create_driver_process(starter_config) == BGS_OK) {
      driver_pid = common::pidof(
        splited_file_name(starter_config.driver_process_path.c_str()));
      PRINT_INFO_FMT("driver pid : %d", driver_pid);
    } else {
      PRINT_ERROR("failed to create driver process");
      driver_pid = common::pidof(
        splited_file_name(starter_config.driver_process_path.c_str()));
      PRINT_INFO_FMT("driver pid : %d", driver_pid);
      if (driver_pid > 0)
          kill(driver_pid, SIGINT);
      destroy_shm();
      return -1;
    }
  } else
    PRINT_INFO("driver process is not running because the config is set so.");

  // step2 run carto
  if (current_run_config.run_carto) {
    if (create_carto_process(starter_config) == BGS_OK) {
      carto_pid = common::pidof(
        splited_file_name(starter_config.carto_process_path.c_str()));
      PRINT_INFO_FMT("carto pid : %d", carto_pid);
    } else {
      PRINT_ERROR("failed to create carto process");
      destroy_shm();
      return -1;
    }
  } else
    PRINT_INFO("carto process is not running because the config is set so.");

  // step3 run controller
  if (current_run_config.run_controller) {
    if (create_controller_process(starter_config) == BGS_OK) {
      controller_pid = common::pidof(
        splited_file_name(starter_config.controller_process_path.c_str()));
      PRINT_INFO_FMT("controller pid : %d", controller_pid);
    } else {
      PRINT_ERROR("failed to create controller process");
      destroy_shm();
      return -1;
    }
  } else
    PRINT_INFO(
      "controller process is not running because the config is set so.");

  // step4 run user module (optional)

  while (true) {
    // refresh all pids
    driver_pid = common::pidof(
      splited_file_name(starter_config.driver_process_path.c_str()));
    carto_pid = common::pidof(
      splited_file_name(starter_config.carto_process_path.c_str()));
    controller_pid = common::pidof(
      splited_file_name(starter_config.controller_process_path.c_str()));

    //     if (driver_pid <= 0 && carto_pid <= 0 )
    //       current_system_state = system_starter::kIdle;

    if (!run_heart_beat) {
      // wait all processes to exit
      if (driver_pid <= 0 && carto_pid <= 0 && controller_pid <= 0) {
        PRINT_INFO("all processes exited.");
        break;
      } else {
        msleep(200);
        continue;
      }
    }

    msleep(200);
  }

  // server thread exit
  PRINT_DEBUG("close all connections");
  for (auto &connection : echo.get_connections())
    connection->send_close(1000);

  msleep(500);
  PRINT_DEBUG("stop the server");
  server.stop();

  websocket_server_thread.join();
  PRINT_DEBUG("websocket thread join");
  
  PRINT_DEBUG("destroy shared memory");

  common::reset_lock_file(lock_fd);
  destroy_shm();
  return 0;
}

int32_t shm_id = 0;
int32_t create_and_init_shm()
{
  // 指定 key 是为了每次启动时 key 的统一性，避免出现在不同的进程里 ftok 获取的
  // key 不同
  int32_t shm_key = BGS_SHM_KEY;

  const int32_t size_of_shm = sizeof(GlobalShm);
  shm_id = shmget(shm_key, size_of_shm, 0666);
  if (shm_id != -1) //  首先检查共享内存是否存在，存在则先删除
  {
    g_ptr_shm = (GlobalShm *)shmat(shm_id, NULL, 0);
    if (g_ptr_shm != (void *)-1) {
      shmdt(g_ptr_shm);
      shmctl(shm_id, IPC_RMID, 0);
    }
  }

  // step1 get shared memory
  shm_id = shmget(shm_key, size_of_shm, 0666 | IPC_CREAT | IPC_EXCL);
  if (shm_id == -1) {
    starter_status.shm_init_succeed = FALSE;
    switch (errno) {
    case EEXIST:
      PRINT_ERROR("shared memory has been existed!");
      return BGS_ERROR;

    case EINVAL:
      PRINT_ERROR("Shm get failed! ( size )");
      return BGS_ERROR;

    default:
      PRINT_ERROR("Shm get failed! ( default )");
      return BGS_ERROR;
    }
  } else {
    starter_status.shm_init_succeed = TRUE;
    PRINT_INFO_FMT("shm key = %x, size = %d ", shm_key, size_of_shm);
  }

  // step2 semophares
  g_ptr_shm = (GlobalShm *)shmat(shm_id, NULL, 0);
  memset(g_ptr_shm, 0, sizeof(GlobalShm));

  // shared memory locker ( semophare )
  g_ptr_shm->shm_sem_id = semget(BGS_SHM_SEM_KEY, 1, 0666 | IPC_CREAT);
  if (g_ptr_shm->shm_sem_id == -1 || init_sem(g_ptr_shm->shm_sem_id, 1) < 0) {
    starter_status.shm_sem_init_succeed = FALSE;
    destroy_shm();
    return BGS_ERROR;
  }
  starter_status.shm_sem_init_succeed = TRUE;
  PRINT_INFO("shm semaphore init succeed!");

  // carto sem
  int32_t local_carto_sem_id = semget(BGS_CARTO_SEM_KEY, 1, 0666 | IPC_CREAT);
  if (local_carto_sem_id == -1 || init_sem(local_carto_sem_id, 0) < 0) {
    starter_status.carto_sem_init_succeed = FALSE;
    destroy_shm();
    return BGS_ERROR;
  }
  starter_status.carto_sem_init_succeed = TRUE;
  g_ptr_shm->carto_sem_id = local_carto_sem_id;
  PRINT_INFO("carto semaphore init succeed!");

  // imu sem
  int32_t local_imu_sem_id = semget(BGS_IMU_SEM_KEY, 1, 0666 | IPC_CREAT);
  if (-1 == local_imu_sem_id || init_sem(local_imu_sem_id, 0) < 0) {
    starter_status.imu_sem_init_succeed = FALSE;
    destroy_shm();
    return BGS_ERROR;
  }
  starter_status.imu_sem_init_succeed = TRUE;
  g_ptr_shm->imu_sem_id = local_imu_sem_id;
  PRINT_INFO("imu semaphore init succeed!");

  // gps sem
  int32_t local_gps_sem_id = semget(BGS_GPS_SEM_KEY, 1, 0666 | IPC_CREAT);
  if (-1 == local_gps_sem_id || init_sem(local_gps_sem_id, 0) < 0) {
    starter_status.gps_sem_init_succeed = FALSE;
    destroy_shm();
    return BGS_ERROR;
  }
  starter_status.gps_sem_init_succeed = TRUE;
  g_ptr_shm->gps_sem_id = local_gps_sem_id;
  PRINT_INFO("gps semaphore init succeed!");

  return BGS_OK;
}

void destroy_shm()
{
  if (!starter_status.shm_init_succeed || !g_ptr_shm)
    return;

  PRINT_INFO("delete semphores!");
  if (starter_status.shm_sem_init_succeed) {
    del_semvalue(g_ptr_shm->shm_sem_id);
    starter_status.shm_sem_init_succeed = FALSE;
  }
  if (starter_status.carto_sem_init_succeed) {
    del_semvalue(g_ptr_shm->carto_sem_id);
    starter_status.carto_sem_init_succeed = FALSE;
  }
  if (starter_status.imu_sem_init_succeed) {
    del_semvalue(g_ptr_shm->imu_sem_id);
    starter_status.imu_sem_init_succeed = FALSE;
  }
  if (starter_status.gps_sem_init_succeed) {
    del_semvalue(g_ptr_shm->gps_sem_id);
    starter_status.gps_sem_init_succeed = FALSE;
  }

  PRINT_INFO("Start destroying shm");
  // system("ipcs -m");

  if (shmdt(g_ptr_shm) == -1) {
    perror("[Liuyc] detach error :");
  }
  if (shmctl(shm_id, IPC_RMID, NULL) == -1) {
    perror("[Liuyc] delete error :");
  }
  g_ptr_shm = NULL;
  // system("ipcs -m");//  此时可看到有进程关联到共享内存的信息，nattch为 0
  PRINT_INFO("Finished destroying shm");
}


void ctrl_c_handler(int32_t sig)
{
  if(flag_ctrl_handler == 0)
  {
    PRINT_DEBUG("Too many Ctrl-C,handle only once");
    PRINT_COLOR(YELLOW, "\nWhich process do you want to kill:");
    std::cout << "    all  -- all processes\n"
              << "    dr -- driver\n"
              << "    ca -- carto\n"
              << "    co -- controller\n"
              << "    no -- nothing\n"
              << "    [all/dr/ca/co/no]:\n";
    return;
  }

  if(flag_ctrl_handler == 1)
  {
    flag_ctrl_handler = 0;
    PRINT_COLOR(YELLOW, "\nWhich process do you want to kill:");
    std::cout << "    all  -- all processes\n"
              << "    dr -- driver\n"
              << "    ca -- carto\n"
              << "    co -- controller\n"
              << "    no -- nothing\n"
              << "    [all/dr/ca/co/no]:\n";
    std::string str_choice;
    getline(std::cin, str_choice);

    if (str_choice.empty()) {
      PRINT_COLOR(RED, "[warning] do nothing because you have typed nothing, it "
                       "you want to interrupt the process, please \"CTRL+C\" "
                       "again.");
      return;
    }

    if (strstr(str_choice.c_str(), "all")) {
      PRINT_COLOR(BOLD, "choice: all processes.");
      if (driver_pid > 0)
        kill(driver_pid, SIGINT);
      if (carto_pid > 0)
        kill(carto_pid, SIGINT);
      if (controller_pid > 0)
        kill(controller_pid, SIGINT);

      run_heart_beat = false;
      return;
    } else if (strstr(str_choice.c_str(), "no")) {
      PRINT_COLOR(BOLD, "do nothing at all.");
    } else {
      if (strstr(str_choice.c_str(), "dr")) {
        PRINT_COLOR(BOLD, "choice: driver.");
        if (driver_pid > 0)
          kill(driver_pid, SIGINT);
      }
      if (strstr(str_choice.c_str(), "ca")) {
        PRINT_COLOR(BOLD, "choice: carto.");
        if (carto_pid > 0)
          kill(carto_pid, SIGINT);
      }
      if (strstr(str_choice.c_str(), "co")) {
        PRINT_COLOR(BOLD, "choice: controller.");
        if (controller_pid > 0)
          kill(controller_pid, SIGINT);
      }
    }
    flag_ctrl_handler = 1;
  }
}

int32_t init_singals()
{
  if (signal(SIGINT, ctrl_c_handler) == SIG_ERR) {
      perror("[Liuyc] signal error: ");
      return BGS_ERROR;
  }
  
  return BGS_OK;
}

int32_t read_configs(int argc, char **argv, system_starter::Config &config)
{
  if (argc <= 1) {
    PRINT_ERROR("you should use the starter like this: \n\n    ./starter "
                "config_file_for_starter\n");
    return BGS_ERROR;
  }

  const char *filename = argv[1];
  pugi::xml_document doc;
  if (!doc.load_file(filename)) // 0代表 load 成功，没有错误
  {
    PRINT_ERROR_FMT("can not load file: %s", filename);
    return BGS_ERROR;
  }

  pugi::xml_node starter_node = doc.child("STARTER");
  if (starter_node.empty()) {
    PRINT_ERROR("Node empty!");
    return BGS_ERROR;
  }

  config.server_port = starter_node.attribute("server_port").as_int();
  config.all_process_run_background =
    starter_node.attribute("all_process_run_background").as_bool();
  config.carto_process_path =
    starter_node.attribute("carto_process_path").as_string();
  config.controller_process_path =
    starter_node.attribute("controller_process_path").as_string();
  config.driver_process_path =
    starter_node.attribute("driver_process_path").as_string();

  const char *node_name[system_starter::kStateCount] = {
    "Default", "State_Mapping", "State_GatheringBaundry", "State_Transition"
  };
  for (int i = 0; i < (int)system_starter::kStateCount; ++i) {
    pugi::xml_node state_config_node = starter_node.child(node_name[i]);
    config.state_run_config[i].run_driver =
      state_config_node.attribute("run_driver").as_bool();
    config.state_run_config[i].run_carto =
      state_config_node.attribute("run_carto").as_bool();
    config.state_run_config[i].run_controller =
      state_config_node.attribute("run_controller").as_bool();

    config.state_run_config[i].driver_config_filename =
      state_config_node.attribute("driver_config_filename").as_string();
    config.state_run_config[i].carto_config_filename =
      state_config_node.attribute("carto_config_filename").as_string();
    config.state_run_config[i].controller_config_filename =
      state_config_node.attribute("controller_config_filename").as_string();

    if (config.state_run_config[i].carto_config_filename.empty()
        || access(config.state_run_config[i].carto_config_filename.c_str(),
                  F_OK)
             < 0
        // 		|| config.state_run_config[i].controller_config_filename.empty()
        // 			|| access( config.controller_config_filename.c_str(), F_OK ) < 0
        || config.state_run_config[i].driver_config_filename.empty()
        || access(config.state_run_config[i].driver_config_filename.c_str(),
                  F_OK)
             < 0) {
      PRINT_ERROR_FMT(
        "config is imcomplete or the config file does not exist! %s",
        node_name[i]);
      return BGS_ERROR;
    }
  }

  int32_t n_start_state = starter_node.attribute("start_state").as_int();
  if (n_start_state >= system_starter::kStateCount || n_start_state < 0) {
    PRINT_WARNING("use the default config");
    n_start_state = 0;
  }
  config.start_state = (system_starter::SystemState)n_start_state;
  current_run_config = config.state_run_config[config.start_state];

  if (config.carto_process_path.empty()
      || access(config.carto_process_path.c_str(), F_OK) < 0) {
    PRINT_ERROR("carto process is not empty or not accessable!");
    return BGS_ERROR;
  }
  if (config.driver_process_path.empty()
      || access(config.driver_process_path.c_str(), F_OK) < 0) {
    PRINT_ERROR("driver process is not empty or not accessable!");
    return BGS_ERROR;
  }
  // 	if( config.controller_process_path.empty() ||
  // access(config.controller_process_path.c_str(), F_OK) < 0)
  // 	{
  // 		PRINT_ERROR( "controller process is not empty or not accessable!" );
  // 		return BGS_ERROR;
  // 	}

  return BGS_OK;
}

inline int32_t create_process_background(std::string cmdline)
{
  return system(("setsid " + cmdline + " &").c_str());
}

inline int32_t create_process_foreground(std::string cmdline)
{
  return system(
    ("gnome-terminal -x bash -c \"" + cmdline + ";exec bash\"").c_str());
}

int32_t create_process(const system_starter::Config &config,
                       std::string cmd_path, std::string cmd_cfg_filename,
                       int8_t &cmd_ready, int32_t max_time)
{
  std::string cmd = cmd_path + " " + cmd_cfg_filename;
  int32_t status = config.all_process_run_background
                     ? create_process_background(cmd)
                     : create_process_foreground(cmd);

  if (status == -1) {
    PRINT_ERROR("system error");
    return BGS_ERROR;
  }
  int32_t time = 0;
  do /* wait for cmd ready */
  {
    sleep(1);
    time++;
    if (time >= max_time) {
      PRINT_ERROR("create process time out!");
      return BGS_ERROR;
    }
    if (!run_heart_beat) {
      PRINT_ERROR("canceled creating process!");
      return BGS_ERROR;
    }

  } while (cmd_ready == FALSE);

  return BGS_OK;
}

int32_t create_driver_process(system_starter::Config &config)
{
  return create_process(config, config.driver_process_path,
                        current_run_config.driver_config_filename,
                        g_ptr_shm->driver_status.all_ready, 60);
}


int32_t create_carto_process(system_starter::Config &config)
{

  // std::cout<<"[debug]start anyway!\n";  
  // create_process_foreground("carto");
  // return BGS_OK;
  return create_process(config, config.carto_process_path,
                        current_run_config.carto_config_filename,
                        g_ptr_shm->driver_status.all_ready, 60);
}

int32_t create_controller_process(system_starter::Config &config)
{

  // std::cout<<"[debug]start anyway!\n";  
  // create_process_foreground("controller");
  // return BGS_OK;
  return create_process(config, config.controller_process_path,
                        current_run_config.controller_config_filename,
                        g_ptr_shm->driver_status.all_ready, 60);
}

int32_t server_handle_msg(const std::string &message,
                          std::shared_ptr<WsServer::Connection> &connection)
{
  auto send_stream = make_shared<WsServer::SendStream>();
  PRINT_DEBUG_FMT("%s", message.c_str());
  if (strstr(message.c_str(), "scan")) {
    if (current_system_state != system_starter::kIdle) {
      *send_stream << "failed";
      connection->send(send_stream);
      return -1;
    }
    PRINT_DEBUG("start mapping ...");
    current_run_config =
      starter_config.state_run_config[system_starter::kMapping];
    current_system_state = system_starter::kMapping;

  } else if (strstr(message.c_str(), "map")) {

    if (current_system_state != system_starter::kMapping) {
      *send_stream << "failed";
      connection->send(send_stream);
      return -1;
    }

    PRINT_DEBUG("finish mapping ...");
    kill(driver_pid, SIGINT);
    kill(carto_pid, SIGINT);

    do {
      msleep(1000);
    } while (driver_pid > 0 && carto_pid > 0);

    sleep(1);
    system("python /home/bongos/imageSave.py");
    sleep(1);

    *send_stream << "done";
    connection->send(send_stream);

    auto map_path_stream = make_shared<WsServer::SendStream>();
    char current_path[256] = { '\0' };
    if (getcwd(current_path, 256) != NULL) {
      std::string image_path = std::string(current_path) + "/image_file.pgm";
      *map_path_stream << image_path;
    } else
      *map_path_stream << "/home/bongos/cart_cpp_test/image_file.pgm";

    connection->send(map_path_stream);
    current_system_state = system_starter::kIdle;
    return 0;
  } else {
  }

  if (current_run_config.run_driver) {
    if (create_driver_process(starter_config) == BGS_OK) {
      driver_pid = common::pidof(
        splited_file_name(starter_config.driver_process_path.c_str()));
      PRINT_INFO_FMT("driver pid : %d", driver_pid);
    } else {
      PRINT_ERROR("failed to create driver process");
      destroy_shm();
      return -1;
    }
  } else
    PRINT_INFO("driver process is not running because the config is set so.");

  // step2 run carto
  if (current_run_config.run_carto) {
    if (create_carto_process(starter_config) == BGS_OK) {
      carto_pid = common::pidof(
        splited_file_name(starter_config.carto_process_path.c_str()));
      PRINT_INFO_FMT("carto pid : %d", carto_pid);
    } else {
      PRINT_ERROR("failed to create carto process");
      destroy_shm();
      return -1;
    }
  } else
    PRINT_INFO("carto process is not running because the config is set so.");

  return 0;
}

int32_t system_state_machine(system_starter::SystemState state)
{
  if (current_system_state == state) {
    PRINT_INFO("no need to change state");
    return 0;
  }

  switch (state) {
  case system_starter::kIdle:
    break;

  case system_starter::kMapping:
    break;

  case system_starter::kGatheringBaundry:
    break;

  case system_starter::kTransition:
    if (driver_pid <= 0 && carto_pid <= 0)
      current_system_state = target_system_state;

    break;
  default:
    break;
  }

  return 0;
}