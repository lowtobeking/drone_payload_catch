#include <vrpn_Tracker.h>
#include <vrpn_Connection.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <thread>
static volatile std::sig_atomic_t running=1;
static void stop(int) { running=0; }
struct Samples { unsigned long received=0, invalid=0; };
void VRPN_CALLBACK pose_cb(void *data,const vrpn_TRACKERCB pose) {
    auto &s=*static_cast<Samples*>(data);
    ++s.received;
    bool valid=true;
    double norm=0;
    for(int i=0;i<3;++i) valid=valid && std::isfinite(pose.pos[i]);
    for(int i=0;i<4;++i) { valid=valid && std::isfinite(pose.quat[i]); norm+=pose.quat[i]*pose.quat[i]; }
    if(!valid || std::abs(norm-1)>0.04) { ++s.invalid; return; }
    const std::int64_t stamp=std::int64_t(pose.msg_time.tv_sec)*1000000+pose.msg_time.tv_usec;
    std::cout<<std::setprecision(17)<<"{\"tracker\":\"planeA\",\"sensor\":"<<pose.sensor
      <<",\"server_stamp_us\":"<<stamp<<",\"position\":["
      <<pose.pos[0]<<","<<pose.pos[1]<<","<<pose.pos[2]<<"],\"quaternion_xyzw\":["
      <<pose.quat[0]<<","<<pose.quat[1]<<","<<pose.quat[2]<<","<<pose.quat[3]<<"]}"<<std::endl;
}
int main(int argc,char **argv) {
    if(argc!=2) { std::cerr<<"usage: planeA_stream LOCAL_WIFI_IP\n"; return 2; }
    std::signal(SIGINT,stop);
    std::signal(SIGTERM,stop);
    auto *conn=vrpn_get_connection_by_name("192.168.1.111:3883",nullptr,nullptr,nullptr,nullptr,argv[1],true);
    if(!conn) return 3;
    Samples samples;
    {
        vrpn_Tracker_Remote tracker("planeA@192.168.1.111:3883",conn);
        tracker.register_change_handler(&samples,pose_cb);
        const auto start=std::chrono::steady_clock::now();
        while(running && std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<30) {
            tracker.mainloop();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    conn->removeReference();
    std::cerr<<"{\"tracker\":\"planeA\",\"received\":"<<samples.received<<",\"invalid\":"<<samples.invalid<<"}"<<std::endl;
    return samples.received?0:2;
}
