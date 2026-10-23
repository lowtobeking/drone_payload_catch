// Bounded user-message metadata probe. No FC or clock/configuration API.
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <thread>
static volatile std::sig_atomic_t running=1;
static void stop(int){running=0;}
struct Summary { unsigned long count=0; std::int64_t min_diff=INT64_MAX,max_diff=INT64_MIN,last_stamp=0,last_wall=0; };
struct Probe { vrpn_Connection *connection; std::map<std::string,Summary> summaries; };
static int VRPN_CALLBACK message(void *opaque,vrpn_HANDLERPARAM p){
  auto &probe=*static_cast<Probe*>(opaque);
  const char *type=probe.connection->message_type_name(p.type);
  const char *sender=probe.connection->sender_name(p.sender);
  using namespace std::chrono;
  const auto wall=duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
  const auto stamp=std::int64_t(p.msg_time.tv_sec)*1000000+p.msg_time.tv_usec;
  std::string key=std::string(sender?sender:"unknown")+" / "+(type?type:"unknown");
  auto &s=probe.summaries[key];++s.count;
  s.min_diff=std::min(s.min_diff,stamp-wall);s.max_diff=std::max(s.max_diff,stamp-wall);
  s.last_stamp=stamp;s.last_wall=wall;
  return 0;
}
int main(){
  std::signal(SIGTERM,stop);std::signal(SIGINT,stop);
  auto *conn=vrpn_get_connection_by_name("192.168.1.111:3883",nullptr,nullptr,nullptr,nullptr,"192.168.1.4",true);
  if(!conn)return 2;
  Probe probe{conn,{}};
  if(conn->register_handler(vrpn_ANY_TYPE,message,&probe,vrpn_ANY_SENDER))return 3;
  {
    vrpn_Tracker_Remote tracker("planeA@192.168.1.111:3883",conn);
    const auto start=std::chrono::steady_clock::now();
    while(running && std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<8){
      tracker.mainloop();std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  conn->unregister_handler(vrpn_ANY_TYPE,message,&probe,vrpn_ANY_SENDER);
  conn->removeReference();
  for(const auto &entry:probe.summaries){
    const auto &s=entry.second;
    std::cout<<"{\"sender_and_type\":"<<std::quoted(entry.first)<<",\"count\":"<<s.count
      <<",\"source_minus_callback_wall_us_min\":"<<s.min_diff
      <<",\"source_minus_callback_wall_us_max\":"<<s.max_diff
      <<",\"last_source_stamp_us\":"<<s.last_stamp<<",\"last_callback_wall_us\":"<<s.last_wall<<"}\n";
  }
  return probe.summaries.empty()?2:0;
}
