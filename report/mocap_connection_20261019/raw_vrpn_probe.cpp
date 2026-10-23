#include <vrpn_Tracker.h>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>
#include <cstdint>
struct Samples {
    uint64_t count=0, invalid=0, nonincreasing=0;
    double last_stamp=0;
    double position[3]={0,0,0}, quaternion[4]={0,0,0,0};
    std::chrono::steady_clock::time_point first,last;
};
void VRPN_CALLBACK on_pose(void *userdata,const vrpn_TRACKERCB pose) {
    auto &s=*static_cast<Samples*>(userdata);
    auto now=std::chrono::steady_clock::now();
    if(s.count==0)s.first=now;
    s.last=now;
    const double stamp=pose.msg_time.tv_sec+pose.msg_time.tv_usec*1e-6;
    if(s.count && stamp<=s.last_stamp)++s.nonincreasing;
    s.last_stamp=stamp;
    ++s.count;
    bool valid=true;
    for(int i=0;i<3;++i){s.position[i]=pose.pos[i];valid=valid && std::isfinite(pose.pos[i]);}
    for(int i=0;i<4;++i){s.quaternion[i]=pose.quat[i];valid=valid && std::isfinite(pose.quat[i]);}
    if(!valid)++s.invalid;
}
int main() {
    Samples samples;
    vrpn_Tracker_Remote tracker("450_1@192.168.1.111:3883");
    tracker.register_change_handler(&samples,on_pose);
    auto start=std::chrono::steady_clock::now();
    while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<15.0) {
        tracker.mainloop();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    auto end=std::chrono::steady_clock::now();
    double elapsed=samples.count>1?std::chrono::duration<double>(samples.last-samples.first).count():0;
    std::cout<<std::setprecision(15)<<"{\"tracker\":\"450_1\",\"server\":\"192.168.1.111:3883\","
      <<"\"count\":"<<samples.count<<",\"invalid\":"<<samples.invalid
      <<",\"nonincreasing_timestamps\":"<<samples.nonincreasing
      <<",\"rate_hz\":"<<(elapsed>0?(samples.count-1)/elapsed:0)
      <<",\"last_rx_age_s\":"<<(samples.count?std::chrono::duration<double>(end-samples.last).count():-1)
      <<",\"last_source_timestamp\":"<<samples.last_stamp<<",\"position\":["
      <<samples.position[0]<<","<<samples.position[1]<<","<<samples.position[2]<<"],\"quaternion_xyzw\":["
      <<samples.quaternion[0]<<","<<samples.quaternion[1]<<","<<samples.quaternion[2]<<","<<samples.quaternion[3]
      <<"],\"forwarded_to_px4\":false}"<<std::endl;
    return samples.count>0?0:2;
}
