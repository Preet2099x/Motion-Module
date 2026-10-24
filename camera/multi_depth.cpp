// Run the depth-distance measurement on several RealSense cameras at once.
//
// Build:
//   g++ -O2 -std=c++14 multi_depth.cpp -o multi_depth -lrealsense2 -lpthread
//
// Run:
//   ./multi_depth                 # auto-discover up to 4 cameras (sorted by serial)
//   ./multi_depth SN1 SN2 SN3 SN4 # or pin the camera order by serial number
//
// Output (one line, updated 10x/s):
//   cam0 dist1/dist2 (cm) | cam1 ... | cam2 ... | cam3 ...

#include <librealsense2/rs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

// D555 over DDS offers depth Z16 at: 256x144, 448x252, 640x360, 896x504,
// 1280x720, 1280x800 with fps 5/15/30/60/90 (480x270 is NOT available).
static const int DEPTH_W   = 448;
static const int DEPTH_H   = 252;
static const int DEPTH_FPS = 15;

static std::atomic<bool> g_running(true);

static void on_sigint(int)
{
    g_running = false;
}

static float percentile5(std::vector<uint16_t>& v)
{
    if (v.empty()) return 0.f;

    size_t idx = (v.size() - 1) / 20;
    std::nth_element(v.begin(), v.begin() + idx, v.end());

    return static_cast<float>(v[idx]);
}

static float percentile10(std::vector<uint16_t>& v)
{
    if (v.empty()) return 0.f;

    size_t idx = (v.size() - 1) / 10;
    std::nth_element(v.begin(), v.begin() + idx, v.end());

    return static_cast<float>(v[idx]);
}

struct Cam
{
    std::string serial;
    std::atomic<bool> live{false};
    std::atomic<float> dist1_cm{0.f};
    std::atomic<float> dist2_cm{0.f};
    std::thread th;
};

static void sleep_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Set an option but never let an unsupported option kill the stream.
static void try_set_option(rs2::sensor& s, rs2_option opt, float val,
                           const char* name, int index)
{
    try
    {
        if (s.supports(opt))
            s.set_option(opt, val);
        else
            std::cerr << "\n[cam " << index << "] option " << name
                      << " not supported, skipping" << std::endl;
    }
    catch (const rs2::error& e)
    {
        std::cerr << "\n[cam " << index << "] set " << name
                  << " failed: " << e.what() << std::endl;
    }
}

// One thread per camera. Restarts the stream if it drops.
static void cam_worker(rs2::context& ctx, Cam& cam, int index)
{
    // Stagger startup so four cameras don't negotiate at the same moment
    for (int i = 0; i < index * 10 && g_running; ++i) sleep_ms(100);

    std::vector<uint16_t> valid1;
    std::vector<uint16_t> valid2;

    while (g_running)
    {
        try
        {
            rs2::pipeline pipe(ctx);
            rs2::config cfg;

            cfg.enable_device(cam.serial);
            cfg.enable_stream(
                RS2_STREAM_DEPTH,
                DEPTH_W,
                DEPTH_H,
                RS2_FORMAT_Z16,
                DEPTH_FPS);

            rs2::pipeline_profile profile = pipe.start(cfg);

            auto depth_sensor =
                profile.get_device().first<rs2::depth_sensor>();

            rs2::sensor& base = depth_sensor;
            try_set_option(base, RS2_OPTION_VISUAL_PRESET, 4.f,
                           "VISUAL_PRESET", index);
            try_set_option(base, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.f,
                           "ENABLE_AUTO_EXPOSURE", index);

            float depth_scale = depth_sensor.get_depth_scale();

            while (g_running)
            {
                rs2::frameset frames = pipe.wait_for_frames(5000);
                rs2::depth_frame depth = frames.get_depth_frame();
                if (!depth) continue;

                const int depth_w = depth.get_width();
                const int depth_h = depth.get_height();

                const int c1 = depth_w / 6;
                const int c2 = 5 * depth_w / 6;

                const int r1a = 0;
                const int r2a = static_cast<int>(depth_h * 0.8f);

                const int r1b = r2a;
                const int r2b = depth_h;

                const uint16_t* data =
                    reinterpret_cast<const uint16_t*>(depth.get_data());

                valid1.clear();
                valid2.clear();

                for (int y = r1a; y < r2a; ++y)
                {
                    for (int x = c1; x < c2; ++x)
                    {
                        uint16_t d = data[y * depth_w + x];
                        if (d > 0) valid1.push_back(d);
                    }
                }

                for (int y = r1b; y < r2b; ++y)
                {
                    for (int x = c1; x < c2; ++x)
                    {
                        uint16_t d = data[y * depth_w + x];
                        if (d > 0) valid2.push_back(d);
                    }
                }

                if (valid1.size() < 100 || valid2.size() < 100)
                    continue;

                float dist1_mm = percentile5(valid1) * depth_scale * 1000.f;
                float dist2_mm = percentile10(valid2) * depth_scale * 1000.f;

                cam.dist1_cm = dist1_mm / 10.f;
                cam.dist2_cm = dist2_mm / 10.f;
                cam.live = true;
            }

            pipe.stop();
        }
        catch (const rs2::error& e)
        {
            cam.live = false;
            std::cerr << "\n[cam " << index << " " << cam.serial
                      << "] RealSense Error: " << e.what() << std::endl;
            for (int i = 0; i < 30 && g_running; ++i) sleep_ms(100);
        }
    }

    cam.live = false;
}

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_sigint);

    try
    {
        rs2::context ctx;

        std::vector<std::string> serials;
        for (int i = 1; i < argc; ++i) serials.push_back(argv[i]);

        if (serials.empty())
        {
            // DDS discovery takes a few seconds, so keep polling
            std::cout << "Looking for cameras (up to 20 s)..." << std::endl;
            auto t0 = std::chrono::steady_clock::now();

            while (g_running &&
                   std::chrono::steady_clock::now() - t0 <
                       std::chrono::seconds(20))
            {
                serials.clear();
                for (auto&& dev : ctx.query_devices())
                    serials.push_back(
                        dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

                if (serials.size() >= 4) break;
                sleep_ms(1000);
            }
            std::sort(serials.begin(), serials.end());
        }

        if (serials.empty())
        {
            std::cerr << "No RealSense cameras found." << std::endl;
            return 1;
        }
        if (serials.size() > 4) serials.resize(4);
        if (serials.size() < 4)
            std::cerr << "WARNING: only " << serials.size()
                      << " camera(s) found" << std::endl;

        std::vector<std::unique_ptr<Cam>> cams;
        for (size_t i = 0; i < serials.size(); ++i)
        {
            cams.emplace_back(new Cam());
            cams.back()->serial = serials[i];
            std::cout << "cam" << i << " = " << serials[i] << std::endl;
        }

        for (size_t i = 0; i < cams.size(); ++i)
            cams[i]->th = std::thread(cam_worker, std::ref(ctx),
                                      std::ref(*cams[i]),
                                      static_cast<int>(i));

        std::cout << "Showing dist1/dist2 in cm per camera. Ctrl-C to stop.\n";

        while (g_running)
        {
            std::string line = "\r";
            char buf[96];

            for (size_t i = 0; i < cams.size(); ++i)
            {
                if (cams[i]->live)
                    std::snprintf(buf, sizeof(buf), "cam%zu %6.1f/%6.1f | ",
                                  i, cams[i]->dist1_cm.load(),
                                  cams[i]->dist2_cm.load());
                else
                    std::snprintf(buf, sizeof(buf), "cam%zu   --  /  --   | ",
                                  i);
                line += buf;
            }

            std::cout << line << std::flush;
            sleep_ms(100);
        }

        std::cout << "\nStopping..." << std::endl;
        for (auto& c : cams)
            if (c->th.joinable()) c->th.join();
    }
    catch (const rs2::error& e)
    {
        std::cerr << "\nRealSense Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}