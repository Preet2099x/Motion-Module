// Run the depth-distance measurement on up to four RealSense cameras (front,
// back, left, right) and send camera lock/release codes to the Teensy.
//
// Build:
//   g++ -O2 -std=c++14 multi_depth.cpp -o multi_depth -lrealsense2 -lpthread
//
// Run:
//   ./multi_depth                         # serials from CAM_SERIALS below
//   ./multi_depth SN_F SN_B SN_L SN_R     # or pass them: front back left right
//                                         # ("-" = no camera on that side)
//   (with neither, cameras are auto-discovered, sorted by serial, and
//    assigned front, back, left, right in that order - check the printout)
//
// Lock rule (per camera, cm): LOCK if dist1 < LOCK_DIST1_CM or
// dist2 < LOCK_DIST2_CM; RELEASE once both are back above their threshold
// plus RELEASE_MARGIN_CM. A code is sent only when the state changes (and
// again for every side after the Teensy link reconnects):
//   front 101 lock / 102 release     back  201 lock / 202 release
//   left  301 lock / 302 release     right 401 lock / 402 release
// A camera that is not live sends nothing. If this program exits while a
// side is locked, that lock stays on the Teensy until an emergency stop.
//
// Teensy link: /dev/ttyACM1, else /dev/ttyACM2 (whichever answers "m" with
// "Motion Module"); reconnects automatically.
//
// Console (one line, updated 10x/s):
//   front dist1/dist2 (cm) [LOCK] | back ... | left ... | right ...

#include <librealsense2/rs.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

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
    int side = 0; // index into SIDES
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

// ── Lock rule (cm) ──
static const float LOCK_DIST1_CM     = 30.f;
static const float LOCK_DIST2_CM     = 50.f;
static const float RELEASE_MARGIN_CM = 5.f; // hysteresis so a reading on the line does not flicker

// ── Positions and their Teensy codes ──
struct SideInfo
{
    const char* name;
    int lock_code;
    int free_code;
};

static const SideInfo SIDES[4] = {
    {"front", 101, 102},
    {"back",  201, 202},
    {"left",  301, 302},
    {"right", 401, 402},
};

// Camera serial number for each position, in SIDES order (front, back, left,
// right). Leave "" for a position with no camera. Command-line serials
// override this table.
static const char* const CAM_SERIALS[4] = {
    "261622303624", // front
    "261522307129", // back
    "261622301455", // left
    "261622300841", // right
};

// ── Teensy serial link ──
static const char* const TEENSY_PORTS[] = {"/dev/ttyACM1", "/dev/ttyACM2"};
static const int RECONNECT_S = 2;

static int open_port(const char* path)
{
    int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    termios tio;
    if (tcgetattr(fd, &tio) != 0)
    {
        ::close(fd);
        return -1;
    }
    cfmakeraw(&tio);
    cfsetspeed(&tio, B115200);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0)
    {
        ::close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

// The Teensy answers "m" with "<serial> | Motion Module". Another device
// sitting on the other ttyACM number will not.
static bool probe_teensy(int fd)
{
    if (::write(fd, "m\n", 2) != 2) return false;

    std::string got;
    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::milliseconds(1500))
    {
        char buf[128];
        ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n > 0)
        {
            got.append(buf, static_cast<size_t>(n));
            if (got.find("Motion Module") != std::string::npos) return true;
        }
        else
        {
            sleep_ms(20);
        }
    }
    return false;
}

class TeensyLink
{
public:
    ~TeensyLink() { close_port(); }

    const std::string& port() const { return port_; }
    bool connected() const { return fd_ >= 0; }

    // Connects if needed (at most once per RECONNECT_S). Returns true only on
    // the call that made a fresh connection.
    bool ensure()
    {
        if (fd_ >= 0) return false;

        auto now = std::chrono::steady_clock::now();
        if (tried_ && now - last_try_ < std::chrono::seconds(RECONNECT_S)) return false;
        tried_ = true;
        last_try_ = now;

        for (const char* path : TEENSY_PORTS)
        {
            int fd = open_port(path);
            if (fd < 0) continue;

            if (probe_teensy(fd))
            {
                fd_ = fd;
                port_ = path;
                std::cerr << "\n[teensy] connected on " << port_ << std::endl;
                return true;
            }
            ::close(fd);
        }
        return false;
    }

    // True if the whole line was written. False means "not sent": the caller
    // keeps the state unsent and tries again.
    bool send_code(int code)
    {
        if (fd_ < 0) return false;

        char buf[16];
        int n = std::snprintf(buf, sizeof(buf), "%d\n", code);
        ssize_t w = ::write(fd_, buf, static_cast<size_t>(n));
        if (w == n) return true;
        if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return false;

        std::cerr << "\n[teensy] write failed on " << port_ << ", reconnecting" << std::endl;
        close_port();
        return false;
    }

private:
    void close_port()
    {
        if (fd_ >= 0) ::close(fd_);
        fd_ = -1;
    }

    int fd_ = -1;
    std::string port_;
    bool tried_ = false;
    std::chrono::steady_clock::time_point last_try_;
};

struct SideState
{
    bool known = false;  // has had a live reading
    bool locked = false; // current decision
    bool sent = false;   // current decision delivered to the Teensy
};

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

        // One serial per position (SIDES order); empty = no camera there.
        std::vector<std::string> serials(4);
        for (int i = 0; i < 4; ++i)
            if (CAM_SERIALS[i][0] != '\0') serials[i] = CAM_SERIALS[i];

        if (argc > 1)
        {
            serials.assign(4, "");
            for (int i = 1; i < argc && i <= 4; ++i)
                if (std::strcmp(argv[i], "-") != 0) serials[i - 1] = argv[i];
        }

        bool have_any = false;
        for (auto& sn : serials)
            if (!sn.empty()) have_any = true;

        if (!have_any)
        {
            // DDS discovery takes a few seconds, so keep polling
            std::cout << "Looking for cameras (up to 20 s)..." << std::endl;
            std::vector<std::string> found;
            auto t0 = std::chrono::steady_clock::now();

            while (g_running &&
                   std::chrono::steady_clock::now() - t0 <
                       std::chrono::seconds(20))
            {
                found.clear();
                for (auto&& dev : ctx.query_devices())
                    found.push_back(
                        dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

                if (found.size() >= 4) break;
                sleep_ms(1000);
            }
            std::sort(found.begin(), found.end());

            if (found.empty())
            {
                std::cerr << "No RealSense cameras found." << std::endl;
                return 1;
            }
            if (found.size() > 4) found.resize(4);

            for (size_t i = 0; i < found.size(); ++i) serials[i] = found[i];
            std::cerr << "WARNING: positions assigned by sorted serial (front, back, "
                         "left, right). Set CAM_SERIALS or pass serials to pin them."
                      << std::endl;
        }

        std::vector<std::unique_ptr<Cam>> cams;
        for (int i = 0; i < 4; ++i)
        {
            if (serials[i].empty()) continue;
            cams.emplace_back(new Cam());
            cams.back()->serial = serials[i];
            cams.back()->side = i;
            std::cout << SIDES[i].name << " = " << serials[i] << std::endl;
        }

        for (size_t i = 0; i < cams.size(); ++i)
            cams[i]->th = std::thread(cam_worker, std::ref(ctx),
                                      std::ref(*cams[i]),
                                      static_cast<int>(i));

        std::cout << "dist1/dist2 in cm per camera; lock below "
                  << LOCK_DIST1_CM << "/" << LOCK_DIST2_CM
                  << " cm. Ctrl-C to stop.\n";

        TeensyLink link;
        SideState states[4];

        while (g_running)
        {
            // On (re)connect, push every known side again: the Teensy may
            // have rebooted or cleared its locks (emergency stop).
            if (link.ensure())
                for (auto& st : states) st.sent = false;

            std::string line = "\r";
            char buf[96];

            for (size_t i = 0; i < cams.size(); ++i)
            {
                Cam& cam = *cams[i];
                const SideInfo& side = SIDES[cam.side];
                SideState& st = states[cam.side];

                // A camera that is not live sends nothing and keeps its last state.
                if (!cam.live)
                {
                    std::snprintf(buf, sizeof(buf), "%s   --  /  --   | ", side.name);
                    line += buf;
                    continue;
                }

                const float d1 = cam.dist1_cm;
                const float d2 = cam.dist2_cm;

                bool want;
                if (st.known && st.locked)
                    want = !(d1 >= LOCK_DIST1_CM + RELEASE_MARGIN_CM &&
                             d2 >= LOCK_DIST2_CM + RELEASE_MARGIN_CM);
                else
                    want = (d1 < LOCK_DIST1_CM || d2 < LOCK_DIST2_CM);

                if (!st.known || want != st.locked)
                {
                    st.known = true;
                    st.locked = want;
                    st.sent = false;
                }

                if (!st.sent)
                    st.sent = link.send_code(st.locked ? side.lock_code
                                                       : side.free_code);

                std::snprintf(buf, sizeof(buf), "%s %6.1f/%6.1f %s | ",
                              side.name, d1, d2, st.locked ? "LOCK" : "    ");
                line += buf;
            }

            line += link.connected() ? link.port() : std::string("no teensy link");
            line += "   ";
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
