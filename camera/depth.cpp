#include <librealsense2/rs.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <vector>

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

int main()
{
    try
    {
        rs2::pipeline pipe;

        rs2::config cfg;

        cfg.enable_stream(
            RS2_STREAM_DEPTH,
            480,
            270,
            RS2_FORMAT_Z16,
            15);

        rs2::pipeline_profile profile =
            pipe.start(cfg);

        auto depth_sensor =
            profile.get_device().first<rs2::depth_sensor>();

        depth_sensor.set_option(
            RS2_OPTION_VISUAL_PRESET,
            4.f);

        depth_sensor.set_option(
            RS2_OPTION_ENABLE_AUTO_EXPOSURE,
            1.f);

        float depth_scale =
            depth_sensor.get_depth_scale();

        const int depth_w = 480;
        const int depth_h = 270;

        const int c1 = depth_w / 6;
        const int c2 = 5 * depth_w / 6;

        const int r1a = 0;
        const int r2a = static_cast<int>(depth_h * 0.8f);

        const int r1b = r2a;
        const int r2b = depth_h;

        std::vector<uint16_t> valid1;
        std::vector<uint16_t> valid2;

        auto last_print =
            std::chrono::steady_clock::now();

        while (true)
        {
            auto frames =
                pipe.wait_for_frames();

            auto depth =
                frames.get_depth_frame();

            const uint16_t* data =
                reinterpret_cast<const uint16_t*>(
                    depth.get_data());

            valid1.clear();
            valid2.clear();

            for (int y = r1a; y < r2a; ++y)
            {
                for (int x = c1; x < c2; ++x)
                {
                    uint16_t d =
                        data[y * depth_w + x];

                    if (d > 0)
                        valid1.push_back(d);
                }
            }

            for (int y = r1b; y < r2b; ++y)
            {
                for (int x = c1; x < c2; ++x)
                {
                    uint16_t d =
                        data[y * depth_w + x];

                    if (d > 0)
                        valid2.push_back(d);
                }
            }

            if (valid1.size() < 100 ||
                valid2.size() < 100)
            {
                continue;
            }

            float dist1_mm =
                percentile5(valid1) *
                depth_scale *
                1000.f;

            float dist2_mm =
                percentile10(valid2) *
                depth_scale *
                1000.f;

            auto now =
                std::chrono::steady_clock::now();

            if (std::chrono::duration_cast<
                    std::chrono::milliseconds>(
                    now - last_print)
                    .count() >= 100)
            {
                std::cout
                    << "\rdist1="
                    << dist1_mm / 10.f
                    << " cm   dist2="
                    << dist2_mm / 10.f
                    << " cm      "
                    << std::flush;

                last_print = now;
            }
        }
    }
    catch (const rs2::error& e)
    {
        std::cerr
            << "\nRealSense Error: "
            << e.what()
            << std::endl;

        return 1;
    }
}
