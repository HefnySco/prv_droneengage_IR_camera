#include "ir_camera_facade.hpp"
#include "../de_common/helpers/colors.hpp"
#include "ir_camera_main.hpp"

using namespace de::ir_camera;

// Shape ids used for the hot/cold '+' markers on the de_camera overlay
// (9201 = hot crosshair, 9202 = cold crosshair).
// High values to avoid colliding with shapes set by other senders.
static constexpr uint32_t IRCAMERA_OVERLAY_SHAPE_ID = 9201;

void CIRCamera_Facade::sendHotColdPointsLocation(
    const std::string &target_party_id, const Json_de targets_location) const {
  if (targets_location.empty()) {
    return;
  }

  Json_de message = {{"t", targets_location}};

#ifdef DDEBUG
  std::cout << "tracking:" << targets_location.dump() << std::endl;
#endif
#ifdef DDEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT << "onTrack >> "
            << _LOG_CONSOLE_BOLD_TEXT << targets_location.dump()
            << _NORMAL_CONSOLE_TEXT_ << std::endl;
#endif
  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_TrackingTargetLocation, true);
}

void CIRCamera_Facade::sendIRCameraStatus(
    const std::string &target_party_id, const int status) const {
  de::ir_camera::CIRCameraMain &m_camera_main =
      de::ir_camera::CIRCameraMain::getInstance();

  const uint8_t tracking_camera_direction =
      m_camera_main.getCameraDirection();
  Json_de message = {
      {"a", status},
      {"b", tracking_camera_direction}
  };

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_IR_CAMERA_MI48_STATUS, true);

#ifdef DEBUG
  std::cout << "TrackingStatus:" << status << std::endl;
#endif
}

/**
 * Sends the hot/cold '+' markers to de_camera as two "crosshair" shapes in a
 * single batched message. Coordinates are normalized [0..1] against the
 * output frame with (0,0) top-left; marker_arm is the '+' arm half-length as
 * a fraction of frame height. A full SET_SHAPE (with style) is sent on the
 * first update and every OVERLAY_FULL_RESEND_INTERVAL updates so de_camera
 * recovers the shapes after a restart or a CLEAR; in between, a batched
 * MOVE_SHAPE carries only the centres. MOVE_SHAPE to an unknown id is
 * ignored by de_camera.
 */
void CIRCamera_Facade::sendCameraOverlayHotCold(
    const std::string &target_party_id, const float &hot_x, const float &hot_y,
    const float &cold_x, const float &cold_y, const float &marker_arm) const {
  static constexpr uint32_t OVERLAY_FULL_RESEND_INTERVAL = 150;

  const bool send_full =
      (m_overlay_send_count % OVERLAY_FULL_RESEND_INTERVAL) == 0;
  ++m_overlay_send_count;

  Json_de message;
  if (send_full) {
    // red '+' for the hot point, blue '+' for the cold point
    Json_de shapes = {
        {{"id", IRCAMERA_OVERLAY_SHAPE_ID + 0},
         {"type", "crosshair"},
         {"x1", hot_x},
         {"y1", hot_y},
         {"radius", marker_arm},
         {"thickness", 2},
         {"color", {200, 0, 0, 255}}},
        {{"id", IRCAMERA_OVERLAY_SHAPE_ID + 1},
         {"type", "crosshair"},
         {"x1", cold_x},
         {"y1", cold_y},
         {"radius", marker_arm},
         {"thickness", 2},
         {"color", {0, 0, 200, 255}}}};

    message = {{"a", CAMERA_OVERLAY_ACTION_SET_SHAPE}, {"s", shapes}};
  } else {
    // crosshair centre is x1/y1; x2/y2 are unused by the renderer
    Json_de moves = {
        {{"i", IRCAMERA_OVERLAY_SHAPE_ID + 0},
         {"x1", hot_x}, {"y1", hot_y},
         {"x2", hot_x}, {"y2", hot_y}},
        {{"i", IRCAMERA_OVERLAY_SHAPE_ID + 1},
         {"x1", cold_x}, {"y1", cold_y},
         {"x2", cold_x}, {"y2", cold_y}}};

    message = {{"a", CAMERA_OVERLAY_ACTION_MOVE_SHAPE}, {"m", moves}};
  }

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_CAMERA_OVERLAY_ACTION, true);

#ifdef DEBUG_OVERLAY_MSG
  static int overlay_send_counter = 0;
  if ((overlay_send_counter++ % 30) == 0) {
    std::cout << "overlay send hot/cold: (" << hot_x << "," << hot_y << ") ("
              << cold_x << "," << cold_y << ")" << std::endl;
  }
#endif
}

void CIRCamera_Facade::sendCameraOverlayRemove(
    const std::string &target_party_id) const {
  m_overlay_send_count = 0;

  Json_de message = {
      {"a", CAMERA_OVERLAY_ACTION_REMOVE_SHAPE},
      {"i", {IRCAMERA_OVERLAY_SHAPE_ID + 0, IRCAMERA_OVERLAY_SHAPE_ID + 1}}};

  m_module.sendJMSG(target_party_id, message,
                    TYPE_AndruavMessage_CAMERA_OVERLAY_ACTION, true);

#ifdef DEBUG_OVERLAY_MSG
  std::cout << "overlay send REMOVE ids:" << IRCAMERA_OVERLAY_SHAPE_ID << ","
            << IRCAMERA_OVERLAY_SHAPE_ID + 1 << std::endl;
#endif
}

