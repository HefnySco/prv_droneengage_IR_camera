#ifndef IR_CAMERA_H
#define IR_CAMERA_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

#include "serial_mi48.hpp"
#include "hotspot_detector.hpp"

#define DEF_CAMERA_ORIENTATION_DEG_0 0
#define DEF_CAMERA_ORIENTATION_DEG_90 1
#define DEF_CAMERA_ORIENTATION_DEG_180 2
#define DEF_CAMERA_ORIENTATION_DEG_270 3

namespace de
{
namespace ir_camera
{
class CCallBack_IRCamera {
public:
    // one observation frame per new thermal frame (hotspot / coldspot, obs 6546)
    virtual void onObservations(const de::ai::ObservationFrame& frame) = 0;

    virtual void onIRStatusChanged(const int& status) = 0;
    virtual void onCameraOverlayHotCold(const float& hot_x, const float& hot_y,
                                        const float& cold_x, const float& cold_y,
                                        const float& marker_arm) = 0;
    virtual void onCameraOverlayRemove() = 0;
};

class CIRCamera {
public:
    CIRCamera(CCallBack_IRCamera* callback_camera)
        : m_callback_camera(callback_camera),
          m_output_video_path(""),
          m_output_video_active(false) {
    }

    ~CIRCamera() {
        uninit();
    }

    // Initialization
    bool init(const std::string& thermal_port,
              const std::string& output_video_device,
              uint16_t frames_to_skip_between_messages,
              const std::string& source_video_device = "",
              bool dual_camera_enabled = false,
              int display_mode = 3,
              bool display_enabled = false,
              bool de_camera_draw = false);
    
    bool uninit();
    
    // Control
    void start();
    void stop();
    void pause();
    void setCameraDraw(const bool de_camera_draw);
    
    // Core processing loop (runs in thread)
    void processIRFrames();
    
    // Calibration parameters
    struct CalibrationParams {
        double scale_x = 1.0;
        double scale_y = 1.0;
        int offset_x = 0;
        int offset_y = 0;
        double rotation = 0.0;
        double alpha = 0.5;
    };

    void setCalibrationParams(const CalibrationParams& params) {
        m_calib_params = params;
    }

    // perceive capability record fields (de.cap/1 "perceive" state defaults)
    inline const std::string& getCamId() const { return m_cam_id; }
    inline uint32_t getTargetFps() const { return m_target_fps; }
    inline double getHfov() const { return m_hfov; }
    inline bool getColdspotEnabled() const { return m_hotspot_config.cold_enabled; }

    CalibrationParams& getCalibrationParams() {
        return m_calib_params;
    }

    void saveCalibrationToConfig();
    
    // Rolling Average Filter for temporal smoothing
    class RollingAverageFilter {
    public:
        RollingAverageFilter(size_t N = 3) : N_(N), buffer_() {}
        
        cv::Mat operator()(const cv::Mat& frame) {
            buffer_.push_back(frame.clone());
            if (buffer_.size() > N_) {
                buffer_.erase(buffer_.begin());
            }
            
            cv::Mat sum = cv::Mat::zeros(frame.rows, frame.cols, CV_32F);
            for (const auto& buf : buffer_) {
                sum += buf;
            }
            return sum / buffer_.size();
        }

        void setBufferSize(size_t N) {
            N_ = N;
            buffer_.clear();
        }

    private:
        size_t N_;
        std::vector<cv::Mat> buffer_;
    };
    
private:
    // MI48 camera setup
    bool initThermalCamera(const std::string& thermal_port);
    
    // V4L2 output device setup (similar to camera module)
    bool initTargetVirtualVideoDevice(const std::string& output_video_device);
    void destroyVirtualVideoDevice();
    
    // Thermal frame processing
    void onThermalFrame(const std::vector<float>& temperatures, 
                        const uint16_t rows, 
                        const uint16_t cols);
    
    // Copy the latest temperatures (raw sensor orientation) and the frame counter
    bool latestTemperatures(cv::Mat& temps, uint64_t& seq);

    // Find hot/cold points in thermal data (overlay markers only)
    void findHotColdPoints(const cv::Mat& thermal_frame,
                          cv::Point& hot_point,
                          cv::Point& cold_point,
                          float& max_temp,
                          float& min_temp);
    
    // Convert thermal data to displayable image
    cv::Mat thermalToColorMap(const std::vector<float>& temperatures,
                              uint16_t rows, uint16_t cols);
    
    // Coordinate conversion (pixel to normalized)
    float revScaleX(const float& x) const;
    float revScaleY(const float& y) const;
    void clearCameraOverlayShapes();

    // Maps a point in rotated thermal-pixel space to normalized [0..1]
    // coordinates of the output (streamed) frame, following the active
    // display mode (thermal-only, side-by-side, overlay, pip). rgb_active
    // tells whether the current output frame is a dual-camera combine.
    cv::Point2f mapThermalToOutputPoint(const cv::Point2f& point, const bool rgb_active) const;

private:
    // Dual camera helper methods
    cv::Mat stretchImage(const cv::Mat& image, double scale_x, double scale_y, 
                         int offset_x, int offset_y, double rotation);
    cv::Mat overlayThermalOnRGB(const cv::Mat& rgb_image, const cv::Mat& thermal_image);
    cv::Mat sideBySide(const cv::Mat& rgb_image, const cv::Mat& thermal_image);
    cv::Mat pictureInPicture(const cv::Mat& rgb_image, const cv::Mat& thermal_image, double pip_scale = 0.3);
    bool initRGBCamera(const std::string& source_video_device);
    std::atomic<bool> m_process{false};
    std::string m_thermal_port;
    
    SerialCommandSender m_sender;
    CCallBack_IRCamera* m_callback_camera;
    
    std::string m_output_video_path;
    bool m_output_video_active = false;
    int m_video_fd = -1;
    int m_yuv_frame_size = 0;
    
    bool m_virtual_device_opened = false;
    
    int m_image_width = 640;
    int m_image_height = 480;
    
    int m_thermal_width = 80;
    int m_thermal_height = 62;
    
    uint32_t m_target_fps = 30;
    
    std::thread m_framesThread;
    std::thread m_thermal_thread;
    
    uint16_t m_frames_to_skip_between_messages = 3;

    // hotspot observations
    de::ir_camera::HotspotConfig m_hotspot_config;
    std::string m_cam_id = "DE-THERMAL";
    double m_hfov = 0.0;
    uint64_t m_last_observed_seq = 0;
    
    // Dual camera support
    bool m_dual_camera_enabled = false;
    bool m_display_enabled = false;
    int m_display_mode = 3;  // 1=separate, 2=side-by-side, 3=overlay, 4=pip
    bool m_de_camera_draw = false;
    bool m_overlay_shapes_drawn = false;
    cv::VideoCapture m_rgb_capture;
    std::string m_source_video_device;
    int m_rgb_width = 0;
    int m_rgb_height = 0;
    CalibrationParams m_calib_params;
    
    // Temporal averaging configuration
    bool m_temporal_averaging_enabled = false;
    int m_temporal_smooth_frames = 3;
    RollingAverageFilter m_temporal_filter;
};

}
}

#endif // IR_CAMERA_HPP