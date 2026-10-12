/**
 * @file test_hotspot.cpp
 * @brief standalone tests for src/ir_camera/hotspot_detector.hpp.
 *
 * Build & run:  make -C test && ./test/test_hotspot
 */

#include <cmath>
#include <iostream>
#include <string>

#include "../src/ir_camera/hotspot_detector.hpp"
#include "../src/de_common/helpers/track_manager.hpp"

#define ASSERT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            std::cerr << "ASSERTION FAILED: " << #condition << " at line " << __LINE__ << std::endl; \
            return false; \
        } \
    } while(0)

#define ASSERT_NEAR(a, b, eps) ASSERT_TRUE(std::fabs((a) - (b)) <= (eps))

using de::ir_camera::HotspotConfig;
using de::ir_camera::detectHotspots;

// MI48-like frame: 62 rows x 80 cols, flat 20 C, with optional noise-free blobs
static const int ROWS = 62;
static const int COLS = 80;

static cv::Mat flat (float t = 20.0f) { return cv::Mat(ROWS, COLS, CV_32F, cv::Scalar(t)); }

static void blob (cv::Mat& m, int x0, int y0, int w, int h, float t)
{
    for (int y = y0; y < y0 + h; ++y)
        for (int x = x0; x < x0 + w; ++x)
            m.at<float>(y, x) = t;
}

// the blob's rect (raw pixels) must lie inside the reported box once rotated 90 deg cw
static bool boxContains (const de::ai::Observation& o, int x0, int y0, int w, int h)
{
    const double bx0 = o.x * ROWS, bx1 = (o.x + o.w) * ROWS;     // rotated image: ROWS wide
    const double by0 = o.y * COLS, by1 = (o.y + o.h) * COLS;     // COLS high
    // raw (x,y) -> rotated (ROWS-1-y, x)
    const double rx0 = ROWS - 1 - (y0 + h - 1), rx1 = ROWS - 1 - y0 + 1;
    const double ry0 = x0, ry1 = x0 + w;
    return bx0 <= rx0 + 1e-6 && bx1 >= rx1 - 1e-6 && by0 <= ry0 + 1e-6 && by1 >= ry1 - 1e-6;
}


bool test_one_40c_blob_on_20c ()
{
    std::cout << "Testing: one 40 C blob on 20 C gives exactly one hotspot..." << std::endl;

    cv::Mat t = flat();
    blob(t, 30, 20, 4, 3, 40.0f);

    de::ai::ObservationFrame f = detectHotspots(t, HotspotConfig(), "DE-THERMAL", 12345);

    ASSERT_TRUE(f.cam_id == "DE-THERMAL");
    ASSERT_TRUE(f.tm_capture == 12345);
    ASSERT_TRUE(f.obs.size() == 1);

    const de::ai::Observation& o = f.obs[0];
    ASSERT_TRUE(o.cls == "hotspot");
    ASSERT_NEAR(o.attrs["temp_c"].get<double>(), 40.0, 1e-6);
    ASSERT_TRUE(o.attrs["area_px"].get<int>() == 12);
    ASSERT_TRUE(o.conf > 0.5 && o.conf <= 1.0);
    ASSERT_TRUE(de::ai::obsBoxValid(o));
    ASSERT_TRUE(boxContains(o, 30, 20, 4, 3));

    // and it is exactly the blob (rotated), not a bigger area
    ASSERT_NEAR(o.w * ROWS, 3.0, 1e-6);   // raw height 3 -> width after rotation
    ASSERT_NEAR(o.h * COLS, 4.0, 1e-6);   // raw width 4 -> height after rotation

    // the frame is a valid wire message
    Json_de wire; std::string err;
    ASSERT_TRUE(de::ai::obsBuild(f, wire, err));

    std::cout << "passed" << std::endl;
    return true;
}


bool test_noise_and_thresholds ()
{
    std::cout << "Testing: flat frame, single hot pixel, weak and cool blobs give nothing..." << std::endl;

    ASSERT_TRUE(detectHotspots(flat(), HotspotConfig(), "C", 1).obs.empty());

    cv::Mat t = flat();
    t.at<float>(10, 10) = 60.0f;                       // 1 pixel < min_area_px 2
    ASSERT_TRUE(detectHotspots(t, HotspotConfig(), "C", 1).obs.empty());

    cv::Mat weak = flat();
    blob(weak, 10, 10, 4, 4, 25.0f);                    // +5 C < min_delta_c 8
    ASSERT_TRUE(detectHotspots(weak, HotspotConfig(), "C", 1).obs.empty());

    // absolute floor: a +10 C blob that is still below min_temp_c is ignored
    HotspotConfig floor_cfg; floor_cfg.min_temp_c = 35.0;
    cv::Mat warm = flat();
    blob(warm, 10, 10, 4, 4, 30.0f);
    ASSERT_TRUE(detectHotspots(warm, floor_cfg, "C", 1).obs.empty());
    ASSERT_TRUE(detectHotspots(warm, HotspotConfig(), "C", 1).obs.size() == 1);

    // the background follows the scene: everything at 80 C with one 90 C patch is +10 C
    cv::Mat hot_scene = flat(80.0f);
    blob(hot_scene, 5, 5, 3, 3, 90.0f);
    ASSERT_TRUE(detectHotspots(hot_scene, HotspotConfig(), "C", 1).obs.size() == 1);

    // empty / wrong-type input is harmless
    ASSERT_TRUE(detectHotspots(cv::Mat(), HotspotConfig(), "C", 1).obs.empty());
    ASSERT_TRUE(detectHotspots(cv::Mat(ROWS, COLS, CV_8U, cv::Scalar(0)), HotspotConfig(), "C", 1).obs.empty());

    std::cout << "passed" << std::endl;
    return true;
}


bool test_several_blobs_ranked_and_capped ()
{
    std::cout << "Testing: several blobs, strongest first, capped by max_spots..." << std::endl;

    cv::Mat t = flat();
    blob(t, 5, 5, 3, 3, 35.0f);     // +15
    blob(t, 40, 30, 3, 3, 50.0f);   // +30
    blob(t, 60, 10, 3, 3, 42.0f);   // +22

    de::ai::ObservationFrame f = detectHotspots(t, HotspotConfig(), "C", 1);
    ASSERT_TRUE(f.obs.size() == 3);
    ASSERT_NEAR(f.obs[0].attrs["temp_c"].get<double>(), 50.0, 1e-6);
    ASSERT_NEAR(f.obs[1].attrs["temp_c"].get<double>(), 42.0, 1e-6);
    ASSERT_NEAR(f.obs[2].attrs["temp_c"].get<double>(), 35.0, 1e-6);
    ASSERT_TRUE(f.obs[0].conf >= f.obs[1].conf && f.obs[1].conf >= f.obs[2].conf);
    ASSERT_TRUE(f.obs[0].conf == 1.0);   // contrast 30 >= 3 * 8

    HotspotConfig cap; cap.max_spots = 2;
    ASSERT_TRUE(detectHotspots(t, cap, "C", 1).obs.size() == 2);

    // a blob that touches the sensor border still yields a valid box
    cv::Mat edge = flat();
    blob(edge, COLS - 3, ROWS - 3, 3, 3, 45.0f);
    de::ai::ObservationFrame e = detectHotspots(edge, HotspotConfig(), "C", 1);
    ASSERT_TRUE(e.obs.size() == 1);
    ASSERT_TRUE(de::ai::obsBoxValid(e.obs[0]));
    ASSERT_TRUE(boxContains(e.obs[0], COLS - 3, ROWS - 3, 3, 3));

    std::cout << "passed" << std::endl;
    return true;
}


bool test_coldspots ()
{
    std::cout << "Testing: coldspot only when enabled..." << std::endl;

    cv::Mat t = flat();
    blob(t, 20, 20, 4, 4, 5.0f);    // -15 C

    ASSERT_TRUE(detectHotspots(t, HotspotConfig(), "C", 1).obs.empty());

    HotspotConfig c; c.cold_enabled = true;
    de::ai::ObservationFrame f = detectHotspots(t, c, "C", 1);
    ASSERT_TRUE(f.obs.size() == 1);
    ASSERT_TRUE(f.obs[0].cls == "coldspot");
    ASSERT_NEAR(f.obs[0].attrs["temp_c"].get<double>(), 5.0, 1e-6);
    ASSERT_TRUE(boxContains(f.obs[0], 20, 20, 4, 4));

    std::cout << "passed" << std::endl;
    return true;
}


bool test_through_track_manager ()
{
    std::cout << "Testing: hotspots become confirmed tracks with a stable trk..." << std::endl;

    de::ai::TrackManagerConfig tc; tc.confirm_hits = 3;
    de::ai::TrackManager tm(tc);

    int64_t id = -1;
    for (int i = 0; i < 6; ++i)
    {
        cv::Mat t = flat();
        blob(t, 30 + i, 20, 4, 3, 40.0f);   // drifting 1 px per frame
        de::ai::ObservationFrame out = tm.update(detectHotspots(t, HotspotConfig(), "DE-THERMAL", i));
        if (i < 2) { ASSERT_TRUE(out.obs.empty()); continue; }
        ASSERT_TRUE(out.obs.size() == 1);
        ASSERT_TRUE(out.obs[0].has_trk);
        if (id < 0) id = out.obs[0].trk;
        ASSERT_TRUE(out.obs[0].trk == id);
        ASSERT_TRUE(out.obs[0].cls == "hotspot");
    }

    std::cout << "passed" << std::endl;
    return true;
}


int main ()
{
    bool ok = true;

    ok &= test_one_40c_blob_on_20c();
    ok &= test_noise_and_thresholds();
    ok &= test_several_blobs_ranked_and_capped();
    ok &= test_coldspots();
    ok &= test_through_track_manager();

    if (ok)
    {
        std::cout << std::endl << "ALL TESTS PASSED" << std::endl;
        return 0;
    }

    std::cout << std::endl << "TESTS FAILED" << std::endl;
    return 1;
}
