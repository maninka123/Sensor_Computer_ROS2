#include <Spinnaker.h>
#include <iostream>

int main() {
  using namespace Spinnaker;
  try {
    auto system = System::GetInstance();
    const auto version = system->GetLibraryVersion();
    std::cout << "Spinnaker " << version.major << "." << version.minor << "."
              << version.type << "." << version.build << "\n";
    auto cameras = system->GetCameras();
    std::cout << "Discovered cameras: " << cameras.GetSize() << "\n";
    for (unsigned i=0; i<cameras.GetSize(); ++i) {
      auto camera = cameras.GetByIndex(i);
      auto & map = camera->GetTLDeviceNodeMap();
      GenApi::CStringPtr serial = map.GetNode("DeviceSerialNumber");
      GenApi::CStringPtr model = map.GetNode("DeviceModelName");
      std::cout << "Camera: serial=" << (GenApi::IsReadable(serial) ? serial->GetValue() : "unknown")
                << " model=" << (GenApi::IsReadable(model) ? model->GetValue() : "unknown") << "\n";
    }
    const bool found = cameras.GetSize()>0;
    cameras.Clear();
    system->ReleaseInstance();
    return found ? 0 : 2;
  } catch (const Spinnaker::Exception & e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
