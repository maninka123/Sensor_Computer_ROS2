// Explicit, serial-selected GigE setup; never run concurrently with acquisition.
#include <Spinnaker.h>
#include <arpa/inet.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <chrono>

uint32_t ip(const char *text) {
  in_addr address{};
  if (inet_pton(AF_INET,text,&address)!=1) throw std::runtime_error("Invalid IPv4 address");
  return ntohl(address.s_addr);
}
std::string dotted(uint32_t value) {
  in_addr address{};address.s_addr=htonl(value);
  return inet_ntoa(address);
}
int main(int argc,char **argv) {
  if(argc!=2 && argc!=3) { std::cerr<<"Usage: camera_network SERIAL [PERSISTENT_IP]\n";return 2; }
  uint32_t target=0;
  if(argc==3) {
    try { target=ip(argv[2]); }
    catch(const std::exception &error) {std::cerr<<error.what()<<std::endl;return 2;}
    if ((target & 255)==0 || (target & 255)==255 || (target>>24)>=224 || (target>>24)==0) {
      std::cerr<<"Use a unicast host address on a /24 sensor subnet\n";return 2;
    }
  }
  using namespace Spinnaker;
  auto system=System::GetInstance();
  auto cameras=system->GetCameras();
  CameraPtr selected;
  try {
    for(unsigned i=0;i<cameras.GetSize();++i) {
      auto camera=cameras.GetByIndex(i);
      GenApi::CStringPtr serial=camera->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
      if(GenApi::IsReadable(serial) && serial->GetValue()==argv[1]) {selected=camera;break;}
    }
    if(!selected) throw std::runtime_error("Requested camera serial not discovered");
    GenApi::CIntegerPtr current=selected->GetTLDeviceNodeMap().GetNode("GevDeviceIPAddress");
    std::cout<<"CurrentIP="<<dotted(current->GetValue())<<std::endl;
    if(argc==3) {
      auto &map=selected->GetTLDeviceNodeMap();
      GenApi::CIntegerPtr address=map.GetNode("GevDeviceForceIPAddress");
      GenApi::CIntegerPtr mask=map.GetNode("GevDeviceForceSubnetMask");
      GenApi::CIntegerPtr gateway=map.GetNode("GevDeviceForceGateway");
      GenApi::CCommandPtr force=map.GetNode("GevDeviceForceIP");
      address->SetValue(target);mask->SetValue(ip("255.255.255.0"));gateway->SetValue(0);
      force->Execute();
      std::cout<<"Applied current IP="<<argv[2]<<std::endl;
      selected=nullptr;cameras.Clear();
      std::this_thread::sleep_for(std::chrono::seconds(2));
      cameras=system->GetCameras();
      for(unsigned i=0;i<cameras.GetSize();++i) {
        auto camera=cameras.GetByIndex(i);
        GenApi::CStringPtr serial=camera->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
        if(GenApi::IsReadable(serial) && serial->GetValue()==argv[1]) {selected=camera;break;}
      }
      if(!selected) throw std::runtime_error("Camera not rediscovered after ForceIP");
    }
    selected->Init();
    if(argc==3) {
      // Blackfly S hides the persistent-address nodes until this mode is enabled.
      selected->GevCurrentIPConfigurationPersistentIP.SetValue(true);
      selected->GevPersistentIPAddress.SetValue(target);
      selected->GevPersistentSubnetMask.SetValue(ip("255.255.255.0"));
      selected->GevPersistentDefaultGateway.SetValue((target & 0xffffff00U) | 1U);
      selected->GevCurrentIPConfigurationDHCP.SetValue(false);
      std::cout<<"Stored persistent IP="<<argv[2]<<std::endl;
    }
    std::cout<<"PersistentEnabled="<<selected->GevCurrentIPConfigurationPersistentIP.GetValue()
             <<" DHCP="<<selected->GevCurrentIPConfigurationDHCP.GetValue();
    if(GenApi::IsReadable(selected->GevPersistentIPAddress))
      std::cout<<" PersistentIP="<<dotted(selected->GevPersistentIPAddress.GetValue());
    std::cout<<std::endl;
    selected->DeInit();
    selected=nullptr;cameras.Clear();system->ReleaseInstance();return 0;
  } catch(const std::exception &error) {
    std::cerr<<error.what()<<std::endl;
    if(selected && selected->IsInitialized()) selected->DeInit();
    selected=nullptr;cameras.Clear();system->ReleaseInstance();return 1;
  }
}
