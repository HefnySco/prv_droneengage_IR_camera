#include <stdio.h>
#include <signal.h>
#include <iostream>
#include <atomic>

#include "./de_common/helpers/colors.hpp"
#include "./de_common/helpers/helpers.hpp"
#include "./de_common/helpers/getopt_cpp.hpp"
#include "./de_common/helpers/util_rpi.hpp"
#include "version.hpp"
#include "defines.hpp"
#include "./de_common/de_databus/messages.hpp"
#include "./de_common/de_databus/configFile.hpp"
#include "./de_common/de_databus/localConfigFile.hpp"
#include "./de_common/de_databus/udpClient.hpp"
#include "./de_common/de_databus/de_module.hpp"
#include "./de_common/de_databus/de_facade_base.hpp"
#include "ir_camera/ir_camera_main.hpp"
#include "ir_camera/ir_camera_andruav_message_parser.hpp"




#define MESSAGE_FILTER {TYPE_AndruavMessage_IR_CAMERA_MI48_ACTION,\
                        TYPE_AndruavMessage_IR_CAMERA_MI48_STATUS,\
                        TYPE_AndruavMessage_CONFIG_ACTION, \
                        TYPE_AndruavMessage_DUMMY}

// This is a timestamp used as instance unique number. if changed then communicator module knows module has restarted.
std::time_t instance_time_stamp;

de::comm::CModule& cModule= de::comm::CModule::getInstance();

std::time_t time_stamp;

bool exit_me = false;
std::atomic<bool> shutdown_in_progress(false);
volatile sig_atomic_t shutdown_requested = 0;

// UAVOS Current PartyID read from communicator
std::string  PartyID;
// UAVOS Current GroupID read from communicator
std::string  GroupID;
std::string  ModuleKey;

int AndruavServerConnectionStatus = SOCKET_STATUS_FREASH;

de::ir_camera::CIRCameraMain& cIRCameraMain = de::ir_camera::CIRCameraMain::getInstance();
de::ir_camera::CIRCameraAndruavMessageParser& cIRCameraAndruavMessageParser = de::ir_camera::CIRCameraAndruavMessageParser::getInstance();


de::CConfigFile& cConfigFile = de::CConfigFile::getInstance();
de::CLocalConfigFile& cLocalConfigFile = de::CLocalConfigFile::getInstance();


void quit_handler( int sig );

/**
 * @brief true when exit status.
 * 
 */
bool m_exit = false;

/**
 * @brief hardware serial number
 * 
 */
static std::string hardware_serial;


/**
 * @brief display version info
 * 
 */
void _versionOnly (void)
{
    std::cout << version_string << std::endl;
}


/**
 * @brief display version info
 * 
 */
void _version (void)
{
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Drone-Engage IR Camera Module " << _INFO_CONSOLE_TEXT << "version " << version_string << _NORMAL_CONSOLE_TEXT_ << std::endl;
}


/**
 * @brief display hardware serial number.
 * 
 */
void _displaySerial (void)
{
    _version ();
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "Serial Number: " << _TEXT_BOLD_HIGHTLITED_ << hardware_serial << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
}

/**
 * @brief display help for -h command argument.
 * 
 */
void _usage(void)
{
   _version ();
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "Options" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "\t--serial:          display serial number needed for registration" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "\t                   -s " << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "\t--config:          name and path of configuration file. default [" << configName << "]" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "\t                   -c ./config.json" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "\t--bconfig:          name and path of configuration file. default [" << localConfigName << "]" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "\t                   -b ./config.local" << _NORMAL_CONSOLE_TEXT_ << std::ends;
    std::cout << std::endl << _INFO_CONSOLE_TEXT << "\t--version:         -v" << _NORMAL_CONSOLE_TEXT_ << std::endl;
}



void onReceive (const char * message, int len, Json_de jMsg)
{
    #ifdef DEBUG        
        std::cout << _INFO_CONSOLE_TEXT << "RX MSG: " << message << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
    
    
    if (std::strcmp(jMsg[INTERMODULE_ROUTING_TYPE].get<std::string>().c_str(),CMD_TYPE_INTERMODULE)==0)
    {
        const Json_de cmd = jMsg[ANDRUAV_PROTOCOL_MESSAGE_CMD];
        const int messageType = jMsg[ANDRUAV_PROTOCOL_MESSAGE_TYPE].get<int>();
    
        if (messageType== TYPE_AndruavModule_ID)
        {
            
            const int status = cmd ["g"].get<int>();
            if (AndruavServerConnectionStatus != status)
            {
                //_onConnectionStatusChanged (status);
            }
            AndruavServerConnectionStatus = status;
            
            return ;
        }
    }
    
    cIRCameraAndruavMessageParser.parseMessage(jMsg, message, len);
    
}

void initSerial()
{
    helpers::CUtil_Rpi::getInstance().get_cpu_serial(hardware_serial);
    hardware_serial.append(get_linux_machine_id());
}

void initArguments (int argc, char *argv[])
{
    int opt;
    const struct GetOptLong::option options[] = {
        {"config",         true,   0, 'c'},
        {"bconfig",        true,   0, 'b'},
        {"serial",         false,  0, 's'},
        {"version",        false,  0, 'v'},
        {"versiononly",    false,  0, 'o'},
        {"help",           false,  0, 'h'},
        {0, false, 0, 0}
    };
    // adding ':' means there is extra parameter needed
    GetOptLong gopt(argc, argv, "c:vh",
                    options);

    /*
      parse command line options
     */
    while ((opt = gopt.getoption()) != -1) {
        switch (opt) {
        case 'c':
            configName = gopt.optarg;
            break;
        case 'b':
            localConfigName = gopt.optarg;
            break;
        case 'v':
            _version();
            exit(0);
            break;
        case 'o':
            _versionOnly();
            exit(0);
        case 's':
            _displaySerial();
            exit(0);
            break;
        case 'h':
            _usage();
            exit(0);
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }
}


void initDEModule(int argc, char *argv[])
{
    const Json_de& jsonConfig = cConfigFile.GetConfigJSON();
    de::CLocalConfigFile& cLocalConfigFile = de::CLocalConfigFile::getInstance();
        
    cModule.defineModule(
        MODULE_CLASS_IR_MI48_CAMERA,
        jsonConfig["module_id"],
        cLocalConfigFile.getStringField("module_key"),
        version_string,
        Json_de::array(MESSAGE_FILTER)
    );

    cModule.addModuleFeatures("");
    cModule.setHardware(hardware_serial, ENUM_HARDWARE_TYPE::HARDWARE_TYPE_CPU);
    cModule.setMessageOnReceive (&onReceive);

    // 0 = auto-detect: CModule::init() picks 8192 for localhost, 1472 for remote.
    int udp_chunk_size = 0;

    if (validateField(jsonConfig, "s2s_udp_packet_size",Json_de::value_t::string))
    {
        udp_chunk_size = std::stoi(jsonConfig["s2s_udp_packet_size"].get<std::string>());
    }
    else
    {
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "WARNING:" << _INFO_CONSOLE_TEXT << " MISSING FIELD " << _ERROR_CONSOLE_BOLD_TEXT_ << "s2s_udp_packet_size " <<  _INFO_CONSOLE_TEXT << "is missing in config file. Auto-detect mode is used." << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
    
    // UDP Server
    cModule.init(jsonConfig["s2s_udp_target_ip"].get<std::string>(),
            std::stoi(jsonConfig["s2s_udp_target_port"].get<std::string>().c_str()),
            jsonConfig["s2s_udp_listening_ip"].get<std::string>() ,
            std::stoi(jsonConfig["s2s_udp_listening_port"].get<std::string>().c_str()),
            udp_chunk_size);

}


void init (int argc, char *argv[]) 
{
	signal(SIGINT,quit_handler);
    signal(SIGTERM,quit_handler);
    
    instance_time_stamp = std::time(nullptr);

    // 1- initialize module
    initArguments (argc, argv);

    // 2- initialize serial
    initSerial();

    // Reading Configuration
    std::cout << std::endl << _SUCCESS_CONSOLE_BOLD_TEXT_ << "=================== " << "STARTING PLUGIN ===================" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    _version();

    std::cout << _LOG_CONSOLE_BOLD_TEXT << std::asctime(std::localtime(&instance_time_stamp)) << instance_time_stamp << _INFO_CONSOLE_BOLD_TEXT<< " seconds since the Epoch" << std::endl;
    

    // Define module features
    
    cConfigFile.initConfigFile (configName.c_str());
    cLocalConfigFile.InitConfigFile (localConfigName.c_str());

    ModuleKey = cLocalConfigFile.getStringField("module_key");
    if (ModuleKey=="")
    {
        ModuleKey = std::to_string(get_time_usec());
        cLocalConfigFile.addStringField("module_key",ModuleKey.c_str());
        cLocalConfigFile.apply();
    }

    cIRCameraMain.init();

    // should be last
    initDEModule (argc,argv);

    // de_ir_camera runs OpenCV thermal + RGB fusion; tighten the generic
    // 500MB/20MB-h defaults to fit this lighter module.
    de::comm::CFacade_Base::getInstance().configureMemoryStatus(400, 15);
}


void uninit ()
{
    cIRCameraMain.uninit();
	m_exit = true;

    // Don't call exit(0) here - let main loop exit naturally
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler( int sig )
{
    (void)sig;
    shutdown_requested = 1;
    exit_me = true;
}

int main (int argc, char *argv[]) 
{
	#ifdef DDEBUG
        std::cout << _INFO_CONSOLE_BOLD_TEXT << " ========================== DDEBUG ENABLED =========================="   << _NORMAL_CONSOLE_TEXT_ << std::endl;
    #endif
	
    // Disable synchronization between C and C++ standard streams
    std::ios_base::sync_with_stdio(false);

    // Optionally, untie cin from cout for further speedup (especially with lots of cin/cout alternating)
    std::cin.tie(nullptr);

    init(argc, argv);

    while (!shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!shutdown_in_progress.exchange(true))
    {
        std::cout << _INFO_CONSOLE_TEXT << std::endl << "TERMINATING AT USER REQUEST" <<  _NORMAL_CONSOLE_TEXT_ << std::endl;
        uninit();
    }
    
    // Clean exit
    return 0;
}