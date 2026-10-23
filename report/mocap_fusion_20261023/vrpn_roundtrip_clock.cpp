// Candidate time mapping experiment only. Not a flight clock authority.
// One reliable ping outstanding, no retransmission, fresh connection, no Tracker
// BaseClass auto-ping. Any lost/late/unsolicited pong prevents valid mapping.
#include <vrpn_Connection.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <thread>
#include <algorithm>
static volatile std::sig_atomic_t running=1;
static void stop(int){running=0;}
static std::int64_t wall_us(){
  using namespace std::chrono;
  return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}
static std::int64_t mono_us(){
  using namespace std::chrono;
  return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}
struct Probe{
  bool pending=false,failed=false;
  std::int64_t t1=0,m1=0;
  int received=0,unsolicited=0;
};
static int VRPN_CALLBACK pong(void *opaque,vrpn_HANDLERPARAM p){
  auto &s=*static_cast<Probe*>(opaque);
  auto t4=wall_us(),m4=mono_us();
  if(!s.pending){++s.unsolicited;return 0;}
  s.pending=false;
  const auto rtt=m4-s.m1;
  const auto source=std::int64_t(p.msg_time.tv_sec)*1000000+p.msg_time.tv_usec;
  bool valid=p.payload_len==0 && rtt>0 && rtt<=100000 && std::llabs((t4-s.t1)-rtt)<=2000;
  if(!valid)s.failed=true;
  ++s.received;
  std::cout<<"{\"kind\":\"pong\",\"sequence\":"<<s.received
    <<",\"valid\":"<<(valid?"true":"false")<<",\"local_send_us\":"<<s.t1
    <<",\"local_receive_us\":"<<t4<<",\"server_stamp_us\":"<<source
    <<",\"rtt_us\":"<<rtt
    <<",\"local_minus_source_lower_us\":"<<s.t1-source
    <<",\"local_minus_source_upper_us\":"<<t4-source<<"}"<<std::endl;
  return 0;
}
int main(){
  std::signal(SIGTERM,stop);std::signal(SIGINT,stop);
  auto *conn=vrpn_get_connection_by_name("192.168.1.111:3883",nullptr,nullptr,nullptr,nullptr,"192.168.1.4",true);
  if(!conn)return 2;
  const auto sender=conn->register_sender("planeA");
  const auto ping_type=conn->register_message_type("vrpn_Base ping_message");
  const auto pong_type=conn->register_message_type("vrpn_Base pong_message");
  Probe probe;
  conn->register_handler(pong_type,pong,&probe,sender);
  const auto start=mono_us();
  auto next=start+1500000;
  int sent=0;
  while(running && mono_us()-start<12000000 && !probe.failed && probe.received<20){
    conn->mainloop();
    const auto now=mono_us();
    if(probe.pending && now-probe.m1>150000){
      probe.failed=true;break;
    }
    if(!probe.pending && now>=next && conn->connected()){
      timeval stamp{};
      vrpn_gettimeofday(&stamp,nullptr);
      probe.t1=wall_us();probe.m1=mono_us();probe.pending=true;
      if(conn->pack_message(0,stamp,ping_type,sender,nullptr,vrpn_CONNECTION_RELIABLE)){
        probe.failed=true;break;
      }
      conn->send_pending_reports();
      ++sent;next=now+300000;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  conn->unregister_handler(pong_type,pong,&probe,sender);
  conn->removeReference();
  std::cout<<"{\"kind\":\"summary\",\"sent\":"<<sent<<",\"received\":"<<probe.received
    <<",\"failed\":"<<(probe.failed?"true":"false")<<",\"unsolicited\":"<<probe.unsolicited
    <<",\"pending\":"<<(probe.pending?"true":"false")
    <<",\"server_timestamp_semantics_verified\":false,\"flight_approval\":false}"<<std::endl;
  return !probe.failed && !probe.pending && probe.received==20 && probe.unsolicited==0 ?0:2;
}
