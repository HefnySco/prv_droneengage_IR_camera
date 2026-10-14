
#include <algorithm>
#include <iostream>

#include "../de_common/de_databus/configFile.hpp"
#include "../de_common/de_databus/messages.hpp"
#include "../de_common/helpers/colors.hpp"
#include "../de_common/helpers/helpers.hpp"
#include "../version.hpp"
#include "ir_camera.hpp"
#include "ir_camera_main.hpp"
#include "ir_camera_facade.hpp"
#include "ir_capabilities.hpp"
#include "video.hpp"

using namespace de::ir_camera;

void CIRCameraMain::loopScheduler() {
  while (!m_exit_thread) {
    // timer each 10m sec.
    wait_time_nsec(0, 10000000);

    m_counter++;

    
    if (m_counter % 500 == 0) { // 5 sec
      de::CConfigFile &cConfigFile = de::CConfigFile::getInstance();
      const bool updated = cConfigFile.fileUpdated();
      if (updated) {
        cConfigFile.reloadFile();

        reloadParametersIfConfigChanged();
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_ "Config file updated"
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
      }
    }
  }
}


bool CIRCameraMain::init() {
  m_exit_thread = false;

  bool res = readConfigParameters();
  if (res == false) {
    return false;
  }

  m_camera = std::make_unique<CIRCamera>(this);

  bool ok = m_camera.get()->init(
      m_source_ir_port_device,
      m_output_video_device,
      m_frames_to_skip_between_messages,
      m_source_video_device,
      m_dual_camera_enabled,
      m_display_mode,
      m_display_enabled,
      m_de_camera_draw);
  if (ok == false) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_
              << "FATAL ERROR:" << _INFO_CONSOLE_TEXT
              << " Failed to initialize IR camera. " << _NORMAL_CONSOLE_TEXT_
              << std::endl;
    return false;
  }

  // Set calibration parameters if dual camera is enabled
  if (m_dual_camera_enabled) {
    CIRCamera::CalibrationParams calib;
    calib.scale_x = m_calib_scale_x;
    calib.scale_y = m_calib_scale_y;
    calib.offset_x = m_calib_offset_x;
    calib.offset_y = m_calib_offset_y;
    calib.rotation = m_calib_rotation;
    calib.alpha = m_calib_alpha;
    m_camera.get()->setCalibrationParams(calib);
  }

  m_camera.get()->start();

  m_scheduler_thread = std::thread{[&]() { loopScheduler(); }};

  return true;
}


void CIRCameraMain::reloadParametersIfConfigChanged() {
    Json_de m_jsonConfig = CConfigFile::getInstance().GetConfigJSON();
    if (!m_jsonConfig.contains("tracking")) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_
              << "FATAL ERROR: " << _INFO_CONSOLE_TEXT
              << CConfigFile::getInstance().getFileName()
              << " does not have field " << _ERROR_CONSOLE_TEXT_ << "[tracking]"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;

    return;
  }

  Json_de tracking = m_jsonConfig["tracking"];
  if (tracking.contains("tracking_camera_direction")) {

    if (m_camera_direction > TRACKING_CAMERA_DIRECTION_UP) {
      std::cout << _ERROR_CONSOLE_BOLD_TEXT_
                << "FATAL ERROR:" << _INFO_CONSOLE_TEXT
                << " invalid tracking_camera_direction: "
                << _ERROR_CONSOLE_TEXT_ << (int)m_camera_direction
                << std::endl
                << "Assuming Default Camera Forward" << _NORMAL_CONSOLE_TEXT_
                << std::endl;
      m_camera_direction = TRACKING_CAMERA_DIRECTION_NONE;
    }
    else
    {
      m_camera_direction = tracking["tracking_camera_direction"].get<uint16_t>();
    }
  }

   std::cout << _LOG_CONSOLE_BOLD_TEXT
            << "Using tracking_camera_direction:" << _INFO_CONSOLE_BOLD_TEXT
            << m_camera_direction << _NORMAL_CONSOLE_TEXT_
            << std::endl;


  m_camera_flipped = false;
  if (tracking.contains("camera_flipped")) {
    m_camera_flipped = tracking["camera_flipped"].get<bool>();
    
    std::cout << _LOG_CONSOLE_BOLD_TEXT
                  << "Using camera_flipped:" << _INFO_CONSOLE_BOLD_TEXT
                  << m_camera_flipped << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
  }

  
  m_camera_orientation = DEF_CAMERA_ORIENTATION_DEG_0;
  if (tracking.contains("camera_orientation")) {
    m_camera_orientation = tracking["camera_orientation"].get<uint16_t>();

    std::cout << _LOG_CONSOLE_BOLD_TEXT
                  << "Using camera_orientation:" << _INFO_CONSOLE_BOLD_TEXT
                  << m_camera_orientation << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
  }

  m_de_camera_draw = false;
  if (tracking.contains("de_camera_draw")) {
    m_de_camera_draw = tracking["de_camera_draw"].get<bool>();

    std::cout << _LOG_CONSOLE_BOLD_TEXT
                  << "Using de_camera_draw:" << _INFO_CONSOLE_BOLD_TEXT
                  << m_de_camera_draw << _NORMAL_CONSOLE_TEXT_
                  << std::endl;
  }

  // m_camera is null during the first read inside init(); the value is
  // passed to CIRCamera::init there. On later config reloads propagate it.
  if (m_camera) {
    m_camera->setCameraDraw(m_de_camera_draw);
  }

}

bool CIRCameraMain::readConfigParameters() {
  Json_de m_jsonConfig = CConfigFile::getInstance().GetConfigJSON();
  if (!m_jsonConfig.contains("tracking")) {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_
              << "FATAL ERROR: " << _INFO_CONSOLE_TEXT
              << CConfigFile::getInstance().getFileName()
              << " does not have field " << _ERROR_CONSOLE_TEXT_ << "[tracking]"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;

    return false;
  } else {
    reloadParametersIfConfigChanged();
  }

  m_source_ir_port_device = "";
  m_source_video_device = "";
  m_output_video_device = "";

  if (m_jsonConfig.contains("camera")) {
    Json_de camera = m_jsonConfig["camera"];

    // Read IR thermal port (direct path only, no name lookup for serial ports)
    if (camera.contains("source_ir_port")) {
      m_source_ir_port_device = camera["source_ir_port"].get<std::string>();
      std::cout << _LOG_CONSOLE_BOLD_TEXT << "Using source_ir_port:"
                << _INFO_CONSOLE_BOLD_TEXT << m_source_ir_port_device
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
    } else {
      std::cout << _ERROR_CONSOLE_BOLD_TEXT_
                << "FATAL ERROR: " << _INFO_CONSOLE_TEXT
                << CConfigFile::getInstance().getFileName()
                << " does not have field " << _ERROR_CONSOLE_TEXT_
                << "[source_ir_port]" << _NORMAL_CONSOLE_TEXT_
                << std::endl;
      return false;
    }

    // Read RGB camera device (for dual camera mode)
    if (camera.contains("source_video_device_name")) {
      const int video_index = CVideo::findVideoDeviceIndex(
          camera["source_video_device_name"].get<std::string>());
      if (video_index != -1) {
        m_source_video_device = "/dev/video" + std::to_string(video_index);
        std::cout << _LOG_CONSOLE_BOLD_TEXT << "Using source_video_device_name:"
                  << _INFO_CONSOLE_BOLD_TEXT << m_source_video_device
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
      }
    }
    
    if (m_source_video_device.empty() && camera.contains("source_video_device")) {
      m_source_video_device = camera["source_video_device"].get<std::string>();
      std::cout << _LOG_CONSOLE_BOLD_TEXT << "Using source_video_device:"
                << _INFO_CONSOLE_BOLD_TEXT << m_source_video_device
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }

    // Try output_video_device_name first (translate name to path)
    if (camera.contains("output_video_device_name")) {
      const int video_index = CVideo::findVideoDeviceIndex(
          camera["output_video_device_name"].get<std::string>());
      if (video_index != -1) {
        m_output_video_device = "/dev/video" + std::to_string(video_index);

        std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_
                  << "Using output_video_device_name:"
                  << _INFO_CONSOLE_BOLD_TEXT << m_output_video_device
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
      }
    }

    // Fallback to direct path if name not found or not specified
    if (m_output_video_device.empty() && camera.contains("output_video_device")) {
      m_output_video_device = camera["output_video_device"].get<std::string>();
      std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_
                << "Using output_video_device:" << _INFO_CONSOLE_BOLD_TEXT
                << m_output_video_device << _NORMAL_CONSOLE_TEXT_
                << std::endl;
    }
    
    // output_video_device is optional - can be empty if only displaying
    if (m_output_video_device.empty()) {
      std::cout << _LOG_CONSOLE_BOLD_TEXT
                << "No output_video_device specified - display-only mode"
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }

  } else {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_
              << "FATAL ERROR:" << _INFO_CONSOLE_TEXT
              << " No camera specified in config.json" << _NORMAL_CONSOLE_TEXT_
              << std::endl;
    return false;
  }

  m_frames_to_skip_between_messages = 3;

  if (!m_jsonConfig.contains("advanced_tracking")) {
    std::cout << _INFO_CONSOLE_BOLD_TEXT
              << "Field not found in config.json: " << _INFO_CONSOLE_TEXT
              << "[advanced_tracking]" << _NORMAL_CONSOLE_TEXT_ << std::endl;
    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_ << "Default Values will be used  "
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
  } else {
    Json_de advanced_tracking = m_jsonConfig["advanced_tracking"];
    if (advanced_tracking.contains("frames_to_skip_between_messages")) {
      m_frames_to_skip_between_messages =
          advanced_tracking["frames_to_skip_between_messages"].get<uint16_t>();
    }
    m_frames_to_skip_between_messages = m_frames_to_skip_between_messages == 0
                                            ? 1
                                            : m_frames_to_skip_between_messages;

    std::cout << _SUCCESS_CONSOLE_BOLD_TEXT_
              << "Field Found: advanced_tracking field found:  "
              << _INFO_CONSOLE_TEXT
              << "Following values will be used:" << _NORMAL_CONSOLE_TEXT_
              << std::endl;
  }

  std::cout << _INFO_CONSOLE_TEXT
            << "frames_to_skip_between_messages: " << _LOG_CONSOLE_BOLD_TEXT
            << m_frames_to_skip_between_messages << _NORMAL_CONSOLE_TEXT_ << std::endl;

  de::ai::TrackManagerConfig track_config;
  if (m_jsonConfig.contains("track") && m_jsonConfig["track"].is_object()) {
    const Json_de& track = m_jsonConfig["track"];
    if (track.contains("confirm_hits") && track["confirm_hits"].is_number_integer()) track_config.confirm_hits = track["confirm_hits"].get<int>();
    if (track.contains("max_misses") && track["max_misses"].is_number_integer())     track_config.max_misses = track["max_misses"].get<int>();
    if (track.contains("iou_min") && track["iou_min"].is_number())                   track_config.iou_min = track["iou_min"].get<double>();
    if (track.contains("min_conf") && track["min_conf"].is_number())                 track_config.min_conf = track["min_conf"].get<double>();
  }
  m_track_manager.setConfig(track_config);

  // Read dual camera configuration
  if (m_jsonConfig.contains("dual_camera")) {
    Json_de dual_camera = m_jsonConfig["dual_camera"];
    
    m_dual_camera_enabled = dual_camera.value("enabled", false);
    m_display_enabled = dual_camera.value("display", false);
    m_display_mode = dual_camera.value("display_mode", 3);
    
    // If no video source device is available, force IR-only mode but keep display for IR
    if (m_source_video_device.empty()) {
      if (m_dual_camera_enabled) {
        std::cout << _ERROR_CONSOLE_BOLD_TEXT_
                  << "WARNING: " << _INFO_CONSOLE_TEXT
                  << "No video source device available (source_video_device or source_video_device_name). "
                  << "Disabling dual_camera mode and using IR-only mode."
                  << _NORMAL_CONSOLE_TEXT_ << std::endl;
      }
      m_dual_camera_enabled = false;
    }
    
    std::cout << _LOG_CONSOLE_BOLD_TEXT
              << "Dual camera enabled: " << _INFO_CONSOLE_BOLD_TEXT
              << (m_dual_camera_enabled ? "true" : "false")
              << ", display: " << (m_display_enabled ? "true" : "false")
              << ", display_mode: " << m_display_mode
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    
    if (dual_camera.contains("calibration")) {
      Json_de calibration = dual_camera["calibration"];
      m_calib_scale_x = calibration.value("scale_x", 1.0);
      m_calib_scale_y = calibration.value("scale_y", 1.0);
      m_calib_offset_x = calibration.value("offset_x", 0);
      m_calib_offset_y = calibration.value("offset_y", 0);
      m_calib_rotation = calibration.value("rotation", 0.0);
      m_calib_alpha = calibration.value("alpha", 0.5);
      
      std::cout << _LOG_CONSOLE_BOLD_TEXT
                << "Calibration params - scale_x: " << m_calib_scale_x
                << ", scale_y: " << m_calib_scale_y
                << ", offset_x: " << m_calib_offset_x
                << ", offset_y: " << m_calib_offset_y
                << ", rotation: " << m_calib_rotation
                << ", alpha: " << m_calib_alpha
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
  }

  // Check for no_display override in camera config (after dual_camera settings)
  if (m_jsonConfig.contains("camera")) {
    Json_de camera = m_jsonConfig["camera"];
    if (camera.contains("no_display") && camera["no_display"].get<bool>()) {
      m_display_enabled = false;
      std::cout << _LOG_CONSOLE_BOLD_TEXT
                << "Display disabled by camera.no_display setting"
                << _NORMAL_CONSOLE_TEXT_ << std::endl;
    }
  }

  return true;
}



bool CIRCameraMain::uninit() {
  // exit thread.
  if (m_exit_thread == true) {
    std::cout << "m_exit_thread == true" << std::endl;
    return true;
  }

  m_exit_thread = true;

  // Wait for scheduler thread with timeout
  if (m_scheduler_thread.joinable()) {
    std::cout << _INFO_CONSOLE_TEXT << "Waiting for scheduler thread to finish..." << _NORMAL_CONSOLE_TEXT_ << std::endl;
    m_scheduler_thread.join();
    std::cout << _SUCCESS_CONSOLE_TEXT_ << "Scheduler thread finished" << _NORMAL_CONSOLE_TEXT_ << std::endl;
  }

  // Uninitialize camera (this will stop thermal thread)
  if (m_camera) {
    m_camera.get()->uninit();
  }

  return true;
}


void CIRCameraMain::enableDetection() {
    // streaming should be always available but when enabled means enable sending hot cold points
    m_ir_status = TrackingTarget_STATUS_TRACKING_ENABLED;
    
    // ACK
    m_ir_camera_facade.sendIRCameraStatus(std::string(""), m_ir_status);
}

void CIRCameraMain::stopDetection() {
    m_ir_status = TrackingTarget_STATUS_TRACKING_STOPPED;
    if (m_camera) {
        m_camera.get()->stop();
    }
    m_track_manager.reset();
    
    m_ir_camera_facade.sendIRCameraStatus(std::string(""), m_ir_status);
}

void CIRCameraMain::pauseDetection() {
    if (m_camera) {
        m_camera.get()->pause();
    }
}


/**
 * Hotspot observations of one thermal frame. Only confirmed tracks leave this
 * module, each with a stable trk; the track manager is the single temporal
 * filter here (one thermal frame in, one observation frame out, possibly empty).
 */
void CIRCameraMain::onObservations(const de::ai::ObservationFrame& frame)
{
  m_ir_camera_facade.sendObservations(std::string(""), m_track_manager.update(frame));
}

void CIRCameraMain::onIRStatusChanged(const int& status)
{
  m_ir_status = status;

  m_ir_camera_facade.sendIRCameraStatus(std::string(""), status);

#ifdef DDEBUG
  std::cout << _INFO_CONSOLE_BOLD_TEXT
            << "onIRStatusChanged:" << _LOG_CONSOLE_BOLD_TEXT
            << std::to_string(m_ir_status) << _NORMAL_CONSOLE_TEXT_
            << std::endl;
#endif
}

/**
 * Called by CIRCamera when de_camera_draw is enabled and the hot/cold '+'
 * markers are updated. Coordinates are normalized [0..1] against the output
 * frame, (0,0) top-left; marker_arm is the '+' arm half-length as a fraction
 * of output frame height.
 */
void CIRCameraMain::onCameraOverlayHotCold(const float& hot_x, const float& hot_y,
                                           const float& cold_x, const float& cold_y,
                                           const float& marker_arm)
{
  m_ir_camera_facade.sendCameraOverlayHotCold(std::string(""), hot_x, hot_y,
                                              cold_x, cold_y,
                                              marker_arm);
}

/**
 * Called by CIRCamera when the hot/cold markers should be removed from the
 * de_camera overlay (detection stopped or draw mode disabled).
 */
void CIRCameraMain::onCameraOverlayRemove()
{
  m_ir_camera_facade.sendCameraOverlayRemove(std::string(""));
}

/**
 * Registers the "perceive" capability advert {cam_id, backend, classes[],
 * fps, hfov} — the de.cap/1 replacement for the dropped 1077 class list —
 * and publishes the same record as live state. Called from initDEModule
 * after defineModule, before registration, so the advert hash rides the ID.
 */
void CIRCameraMain::setupCapabilities()
{
  if (!m_camera) return;

  std::vector<std::string> classes = {"hotspot"};
  if (m_camera.get()->getColdspotEnabled())
  {
    classes.push_back("coldspot");
  }

  const Json_de advert = de::ir_camera::buildPerceiveAdvert(
      "de_ir_camera",
      version_string,
      "thermal",
      m_camera.get()->getCamId(),
      classes,
      static_cast<double>(m_camera.get()->getTargetFps()),
      m_camera.get()->getHfov());

  if (!m_ir_camera_facade.setCapabilities({advert.dump()}))
  {
    std::cout << _ERROR_CONSOLE_BOLD_TEXT_
              << "capability advert rejected for ns perceive"
              << _NORMAL_CONSOLE_TEXT_ << std::endl;
    return;
  }

  m_ir_camera_facade.publishState(
      "perceive",
      de::ir_camera::perceiveStatePayload(
          m_camera.get()->getCamId(),
          "thermal",
          classes,
          static_cast<double>(m_camera.get()->getTargetFps()),
          m_camera.get()->getHfov()),
      true);
}