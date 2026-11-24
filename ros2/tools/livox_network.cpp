// Serial-selected original Livox SDK IP inspection/provisioning, without acquisition.
#include <livox_sdk.h>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

std::string code;
std::atomic<bool> connecting{false},done{false};
std::atomic<int> result{1};
bool change=false,reboot=false;
SetStaticDeviceIpModeRequest request{};
std::string dotted(uint32_t value) { in_addr address{};address.s_addr=value;return inet_ntoa(address); }
void rebooted(livox_status status,uint8_t,uint8_t response,void*) {
  std::cout<<"Reboot status="<<status<<" response="<<int(response)<<std::endl;
  result=(status==kStatusSuccess && response==0)?0:1;done=true;
}
void changed(livox_status status,uint8_t handle,uint8_t response,void*) {
  std::cout<<"SetStaticIp status="<<status<<" response="<<int(response)<<std::endl;
  if(status==kStatusSuccess && response==0 && reboot) {
    if(RebootDevice(handle,500,rebooted,nullptr)!=kStatusSuccess) done=true;
  } else {result=(status==kStatusSuccess && response==0)?0:1;done=true;}
}
void info(livox_status status,uint8_t handle,GetDeviceIpModeResponse *response,void*) {
  if(status!=kStatusSuccess || !response || response->ret_code!=0) {done=true;return;}
  std::cout<<"IP="<<dotted(response->ip_addr)<<" mode="<<int(response->ip_mode)
           <<" mask="<<dotted(response->net_mask)<<" gateway="<<dotted(response->gw_addr)<<std::endl;
  if(change) {
    auto status=SetStaticIp(handle,&request,changed,nullptr);
    if(status!=kStatusSuccess) {std::cerr<<"SetStaticIp request rejected "<<status<<std::endl;done=true;}
  } else {result=0;done=true;}
}
void state(const DeviceInfo *device,DeviceEvent event) {
  if(device && code==device->broadcast_code && event==kEventConnect) {
    std::cout<<"Connected="<<device->broadcast_code<<" address="<<device->ip<<std::endl;
    GetDeviceIpInformation(device->handle,info,nullptr);
  }
}
void broadcast(const BroadcastDeviceInfo *device) {
  if(!device || code!=device->broadcast_code || connecting.exchange(true)) return;
  uint8_t handle=0;auto status=AddLidarToConnect(device->broadcast_code,&handle);
  std::cout<<"Discovered="<<device->broadcast_code<<" IP="<<device->ip<<" connect_status="<<status<<std::endl;
}
int main(int argc,char **argv) {
  if(argc<2 || argc>4) {std::cerr<<"Usage: livox_network BROADCAST_CODE [STATIC_IP [--reboot]]\n";return 2;}
  code=argv[1];change=argc>=3;reboot=argc==4 && std::string(argv[3])=="--reboot";
  if(argc==4 && !reboot) {std::cerr<<"Expected --reboot\n";return 2;}
  if(change) {
    in_addr address{};
    if(inet_pton(AF_INET,argv[2],&address)!=1) {std::cerr<<"Invalid IPv4 address\n";return 2;}
    const uint32_t host=ntohl(address.s_addr);
    if((host & 255)==0 || (host & 255)==255 || (host>>24)>=224 || (host>>24)==0) {
      std::cerr<<"Use a unicast host address on a /24 sensor subnet\n";return 2;
    }
    request.ip_addr=address.s_addr;request.net_mask=inet_addr("255.255.255.0");
    request.gw_addr=htonl((host & 0xffffff00U) | 1U);
  }
  if(!Init()) return 1;
  SetBroadcastCallback(broadcast);SetDeviceStateUpdateCallback(state);
  if(!Start()) {Uninit();return 1;}
  for(int i=0;i<200 && !done;++i) std::this_thread::sleep_for(std::chrono::milliseconds(100));
  Uninit();return result;
}
