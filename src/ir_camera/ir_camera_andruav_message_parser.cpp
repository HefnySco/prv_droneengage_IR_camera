#include <iostream>
#include "../de_common/de_databus/messages.hpp"
#include "ir_camera_andruav_message_parser.hpp"

using namespace de::ir_camera;




void CIRCameraAndruavMessageParser::parseCommand(Json_de &andruav_message, const char *full_message, const int &full_message_length, int messageType, uint32_t permission)
{
    const Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];

    switch (messageType)
    {

    case TYPE_AndruavMessage_IR_CAMERA_MI48_ACTION:
    {

        if (!cmd.contains("a") || !cmd["a"].is_number_integer())
            return;

        switch (cmd["a"].get<int>())
        {

        case TrackingTarget_ACTION_TRACKING_ENABLE:
            CIRCameraMain::getInstance().enableDetection();
            break;

        case TrackingTarget_ACTION_TRACKING_STOP:
            CIRCameraMain::getInstance().stopDetection();
            break;

        case TrackingTarget_ACTION_TRACKING_PAUSE:
            CIRCameraMain::getInstance().pauseDetection();
            break;

        default:
            break;
        }
    }
    break;

    case TYPE_AndruavMessage_IR_CAMERA_MI48_STATUS:
    {
        //TODO: LATER
    }
    break;
    }
}

/**
 * @brief part of parseMessage that is responsible only for
 * parsing remote execute command.
 *
 * @param andruav_message
 */
void CIRCameraAndruavMessageParser::parseRemoteExecute(Json_de &andruav_message)
{
    const Json_de cmd = andruav_message[ANDRUAV_PROTOCOL_MESSAGE_CMD];

    if (!validateField(cmd, "C", Json_de::value_t::number_unsigned))
        return;

    const int remoteCommand = cmd["C"].get<int>();
    std::cout << "cmd: " << remoteCommand << std::endl;
}
