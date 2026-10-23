// Explicit single-tracker VRPN reader; no ROS, PX4, parameter or actuator API.
#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>

static volatile std::sig_atomic_t running = 1;
static void stop(int) { running = 0; }
struct Samples { const char *tracker; unsigned long received=0, invalid=0; };

void VRPN_CALLBACK pose_cb(void *opaque, const vrpn_TRACKERCB pose) {
    auto &s = *static_cast<Samples *>(opaque);
    ++s.received;
    bool valid = pose.sensor == 0;
    double norm = 0;
    for (int i=0; i<3; ++i) valid = valid && std::isfinite(pose.pos[i]);
    for (int i=0; i<4; ++i) {
        valid = valid && std::isfinite(pose.quat[i]);
        norm += pose.quat[i] * pose.quat[i];
    }
    if (!valid || std::abs(norm-1) > .04) { ++s.invalid; return; }
    using namespace std::chrono;
    auto wall = duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
    auto mono = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    const std::int64_t source = std::int64_t(pose.msg_time.tv_sec)*1000000 + pose.msg_time.tv_usec;
    std::cout << std::setprecision(17)
      << "{\"schema\":1,\"tracker\":\"planeA\",\"sensor\":0,\"frame\":\"world\","
      << "\"server_stamp_us\":" << source
      << ",\"receive_wall_us\":" << wall << ",\"receive_monotonic_us\":" << mono
      << ",\"position\":[" << pose.pos[0] << "," << pose.pos[1] << "," << pose.pos[2]
      << "],\"quaternion_xyzw\":[" << pose.quat[0] << "," << pose.quat[1] << ","
      << pose.quat[2] << "," << pose.quat[3] << "]}" << std::endl;
}

int main(int argc, char **argv) {
    if (argc != 5 || std::string(argv[2]) != "planeA") {
        std::cerr << "usage: mocap_vrpn_reader SERVER:PORT planeA LOCAL_IP SECONDS\n";
        return 2;
    }
    double seconds;
    try { seconds = std::stod(argv[4]); }
    catch (...) { return 2; }
    if (!std::isfinite(seconds) || seconds < 1 || seconds > 120) return 2;
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    auto *conn = vrpn_get_connection_by_name(argv[1], nullptr, nullptr, nullptr, nullptr, argv[3], true);
    if (!conn) return 3;
    Samples samples{argv[2], 0, 0};
    {
        std::string name = std::string(argv[2]) + "@" + argv[1];
        vrpn_Tracker_Remote tracker(name.c_str(), conn);
        tracker.register_change_handler(&samples, pose_cb);
        const auto start = std::chrono::steady_clock::now();
        while (running && std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() < seconds) {
            tracker.mainloop();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    conn->removeReference();
    std::cerr << "{\"received\":" << samples.received << ",\"invalid\":" << samples.invalid << "}" << std::endl;
    return samples.received ? 0 : 2;
}
