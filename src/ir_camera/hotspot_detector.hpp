#ifndef IR_HOTSPOT_DETECTOR_H
#define IR_HOTSPOT_DETECTOR_H

/**
 * @brief Thermal blob detector: turns one frame of temperatures into
 * observations (de_common helpers/obs.hpp) of class "hotspot" (and
 * optionally "coldspot"). Pure: OpenCV + the obs types, no sensor, no bus.
 *
 * Input is the sensor's temperature matrix in degrees C, exactly as the
 * MI48 delivers it (rows x cols, CV_32F). The module shows the thermal
 * image rotated 90 degrees clockwise, so every box is reported in that
 * upright frame: normalized, TOP-LEFT origin, 0..1 (the obs convention).
 *
 * Detection:
 *   background  = median temperature of the frame
 *   hot pixel   = temp >= background + min_delta_c  AND  temp >= min_temp_c
 *   cold pixel  = temp <= background - min_delta_c   (only when cold_enabled)
 *   blob        = 8-connected pixels; blobs smaller than min_area_px are noise
 *   conf        = clamp(contrast / (3 * min_delta_c), 0, 1), contrast being
 *                 |peak - background| (a blob exactly at the threshold gets
 *                 0.33, one 3x the threshold or more gets 1.0)
 *   attrs       = { temp_c: peak (hot) or lowest (cold) temperature of the
 *                 blob, area_px }
 * The strongest max_spots blobs per class are reported, strongest first.
 */

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "../de_common/helpers/obs.hpp"

namespace de
{
namespace ir_camera
{

struct HotspotConfig
{
    double min_delta_c  = 8.0;      // contrast above (below) the background
    double min_temp_c   = -273.0;   // absolute floor for hot pixels (e.g. 28 to ignore anything below skin temperature)
    int    min_area_px  = 2;        // smallest blob, in sensor pixels
    int    max_spots    = 5;        // per class
    bool   cold_enabled = false;    // also report "coldspot"
};

namespace hotspot_detail
{
    struct Blob
    {
        double peak = 0.0;
        double contrast = 0.0;
        int x0 = 0, y0 = 0, x1 = 0, y1 = 0;   // inclusive, raw sensor pixels
        int area = 0;
    };

    inline double median (const cv::Mat& temps)
    {
        std::vector<float> v;
        v.reserve((size_t) temps.total());
        for (int r = 0; r < temps.rows; ++r)
        {
            const float* p = temps.ptr<float>(r);
            v.insert(v.end(), p, p + temps.cols);
        }
        const size_t mid = v.size() / 2;
        std::nth_element(v.begin(), v.begin() + mid, v.end());
        return v[mid];
    }

    inline std::vector<Blob> blobs (const cv::Mat& temps, const cv::Mat& mask, const bool hot, const double background, const int min_area)
    {
        cv::Mat labels, stats, centroids;
        const int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

        std::vector<Blob> out;
        for (int i = 1; i < n; ++i)   // 0 is the background label
        {
            Blob b;
            b.area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (b.area < min_area) continue;

            b.x0 = stats.at<int>(i, cv::CC_STAT_LEFT);
            b.y0 = stats.at<int>(i, cv::CC_STAT_TOP);
            b.x1 = b.x0 + stats.at<int>(i, cv::CC_STAT_WIDTH) - 1;
            b.y1 = b.y0 + stats.at<int>(i, cv::CC_STAT_HEIGHT) - 1;

            double peak = hot ? -1e30 : 1e30;
            for (int y = b.y0; y <= b.y1; ++y)
            {
                for (int x = b.x0; x <= b.x1; ++x)
                {
                    if (labels.at<int>(y, x) != i) continue;
                    const double t = temps.at<float>(y, x);
                    peak = hot ? std::max(peak, t) : std::min(peak, t);
                }
            }

            b.peak = peak;
            b.contrast = hot ? peak - background : background - peak;
            out.push_back(b);
        }

        std::sort(out.begin(), out.end(), [](const Blob& a, const Blob& b) { return a.contrast > b.contrast; });
        return out;
    }

    // raw sensor rect -> normalized top-left box of the 90 degrees clockwise rotated image
    // (new_x = rows - 1 - old_y, new_y = old_x; the rotated image is rows wide, cols high)
    inline void toBox (const Blob& b, const int rows, const int cols, de::ai::Observation& o)
    {
        o.x = (double) (rows - 1 - b.y1) / (double) rows;
        o.y = (double) b.x0 / (double) cols;
        o.w = (double) (b.y1 - b.y0 + 1) / (double) rows;
        o.h = (double) (b.x1 - b.x0 + 1) / (double) cols;
    }
}


inline de::ai::ObservationFrame detectHotspots (const cv::Mat& temps, const HotspotConfig& config,
                                                const std::string& cam_id, const uint64_t tm_capture)
{
    de::ai::ObservationFrame frame;
    frame.cam_id = cam_id;
    frame.tm_capture = (int64_t) tm_capture;

    if (temps.empty() || temps.type() != CV_32F) return frame;

    const double bg = hotspot_detail::median(temps);
    const double delta = config.min_delta_c > 0.0 ? config.min_delta_c : 1e-6;

    auto emit = [&](const std::string& cls, const bool hot)
    {
        cv::Mat mask;
        if (hot)
        {
            const double thr = std::max(bg + delta, config.min_temp_c);
            mask = temps >= thr;
        }
        else
        {
            mask = temps <= (bg - delta);
        }

        const std::vector<hotspot_detail::Blob> found = hotspot_detail::blobs(temps, mask, hot, bg, config.min_area_px);

        int count = 0;
        for (const hotspot_detail::Blob& b : found)
        {
            if (count >= config.max_spots) break;

            de::ai::Observation o;
            o.cls = cls;
            o.conf = std::max(0.0, std::min(1.0, b.contrast / (3.0 * delta)));
            hotspot_detail::toBox(b, temps.rows, temps.cols, o);
            o.attrs = Json_de::object({{"temp_c", b.peak}, {"area_px", b.area}});

            if (!de::ai::obsBoxValid(o)) continue;
            frame.obs.push_back(o);
            ++count;
        }
    };

    emit("hotspot", true);
    if (config.cold_enabled) emit("coldspot", false);

    return frame;
}

}
}

#endif
