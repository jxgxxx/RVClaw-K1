// Optional Astra depth-only acquisition. No RGB, ROS control, or pose outputs.
#include <OpenNI.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }
using Clock = std::chrono::steady_clock;
std::string boot_id;
std::string output;
std::uint64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}
std::string prefix(const std::string& status, const std::string& reason) {
  // status and reason are fixed literals controlled here; no device strings.
  std::ostringstream s;
  s << "{\"schema_version\":1,\"status\":\"" << status << "\",\"reason\":\"" << reason
    << "\",\"boot_id\":\"" << boot_id << "\",\"monotonic_ns\":" << now_ns()
    << ",\"frame_id\":\"astra_depth_optical_frame\",\"motion_authority\":false"
       ",\"calibration_status\":\"unverified\"";
  return s.str();
}
void write_state(const std::string& state) {
  const auto pending = output + "." + std::to_string(getpid()) + ".tmp";
  {
    std::ofstream out(pending, std::ios::trunc);
    out << state << '\n';
    out.close();
    if (!out) { std::remove(pending.c_str()); throw std::runtime_error("cannot write depth state"); }
  }
  if (std::rename(pending.c_str(), output.c_str()) != 0) {
    std::remove(pending.c_str()); throw std::runtime_error("cannot replace depth state");
  }
}
void unavailable(const char* why) {
  write_state(prefix("UNAVAILABLE", why) + ",\"regions\":[]}");
}
std::string summarize(const openni::VideoFrameRef& frame, int fps) {
  const int w = frame.getWidth(), h = frame.getHeight();
  if (w <= 0 || w > 640 || h <= 0 || h > 480 || frame.getStrideInBytes() < w*2 ||
      frame.getDataSize() < frame.getStrideInBytes()*h ||
      frame.getVideoMode().getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM)
    throw std::runtime_error("invalid depth frame layout");
  std::ostringstream regions;
  bool usable = false;
  const char* names[] = {"left", "center", "right"};
  // Each region samples the central vertical third. Depth remains optical Z,
  // not base clearance and not an RGB-aligned instrument distance.
  for (int k = 0; k < 3; ++k) {
    std::vector<float> values;
    int total = 0;
    for (int y=h/3; y<2*h/3; y+=4) {
      const auto* row = reinterpret_cast<const std::uint16_t*>(
        static_cast<const char*>(frame.getData()) + y*frame.getStrideInBytes());
      for (int x=(k+1)*w/5; x<(k+2)*w/5; x+=4) {
        ++total;
        if (row[x] >= 600 && row[x] <= 4000) values.push_back(row[x]/1000.0f);
      }
    }
    double coverage = total ? static_cast<double>(values.size())/total : 0;
    bool good = values.size() >= 20 && coverage >= .35;
    usable |= good;
    if (k) regions << ',';
    regions << "{\"name\":\"" << names[k] << "\",\"valid_fraction\":" << coverage;
    if (good) {
      std::sort(values.begin(), values.end());
      regions << ",\"median_z_m\":" << values[values.size()/2]
              << ",\"p10_z_m\":" << values[values.size()/10];
    } else regions << ",\"median_z_m\":null,\"p10_z_m\":null";
    regions << '}';
  }
  std::ostringstream s;
  s << prefix(usable ? "OK" : "INVALID", usable ? "depth_only_uncalibrated" : "insufficient_valid_depth")
    << ",\"frame_index\":" << frame.getFrameIndex() << ",\"device_timestamp_us\":" << frame.getTimestamp()
    << ",\"width\":" << w << ",\"height\":" << h << ",\"sensor_fps\":" << fps
    << ",\"regions\":[" << regions.str() << "]}";
  return s.str();
}
}

int main(int argc, char** argv) {
  if (argc != 3) { std::cerr << "usage: rvclaw_depth_reader OUTPUT_JSON SAMPLE_HZ\n"; return 64; }
  output = argv[1];
  double sample_hz;
  try { sample_hz=std::stod(argv[2]); } catch (...) { return 64; }
  if (!std::isfinite(sample_hz) || sample_hz < 1 || sample_hz > 5) return 64;
  std::ifstream("/proc/sys/kernel/random/boot_id") >> boot_id;
  if (boot_id.empty()) return 70;
  std::signal(SIGTERM, stop); std::signal(SIGINT, stop);
  openni::Device device;
  openni::VideoStream depth;
  int result = 0;
  try {
    unavailable("starting");
    if (openni::OpenNI::initialize() != openni::STATUS_OK)
      throw std::runtime_error("OpenNI initialization failed");
    openni::Array<openni::DeviceInfo> devices;
    openni::OpenNI::enumerateDevices(&devices);
    std::string uri;
    for (int i=0; i<devices.getSize(); ++i) {
      if (devices[i].getUsbVendorId()==0x2bc5 && devices[i].getUsbProductId()==0x0403) {
        if (!uri.empty()) throw std::runtime_error("multiple Astra Pro depth devices; explicit selection required");
        uri=devices[i].getUri();
      }
    }
    if (uri.empty()) {
      unavailable("astra_driver_or_device_unavailable");
      std::cerr << "No native OpenNI driver enumerated Astra Pro 2bc5:0403\n";
      result=69;
    } else {
      if (device.open(uri.c_str())!=openni::STATUS_OK || depth.create(device,openni::SENSOR_DEPTH)!=openni::STATUS_OK)
        throw std::runtime_error("Astra depth open failed; check driver and USB permissions");
      const auto& modes=depth.getSensorInfo().getSupportedVideoModes();
      int selected=-1;
      long cost=0;
      for(int i=0;i<modes.getSize();++i) {
        auto m=modes[i];
        long pixels=static_cast<long>(m.getResolutionX())*m.getResolutionY();
        if(m.getPixelFormat()!=openni::PIXEL_FORMAT_DEPTH_1_MM || m.getResolutionX()<320 ||
           m.getResolutionX()>640 || m.getResolutionY()<240 || m.getResolutionY()>480 ||
           m.getFps()<1 || m.getFps()>30) continue;
        long candidate=pixels*m.getFps();
        if(selected<0 || candidate<cost) { selected=i; cost=candidate; }
      }
      if(selected<0 || depth.setVideoMode(modes[selected])!=openni::STATUS_OK ||
         depth.setMirroringEnabled(false)!=openni::STATUS_OK || depth.start()!=openni::STATUS_OK)
        throw std::runtime_error("no supported bounded depth mode");
      int last_index=-1;
      auto last_fresh=Clock::now(), next=Clock::now();
      while(running) {
        openni::VideoStream* streams[]={&depth}; int changed=-1;
        if(openni::OpenNI::waitForAnyStream(streams,1,&changed,500)!=openni::STATUS_OK) {
          unavailable("frame_timeout");
          if(Clock::now()-last_fresh>std::chrono::seconds(3)) throw std::runtime_error("depth stream stalled");
          continue;
        }
        openni::VideoFrameRef frame;
        if(depth.readFrame(&frame)!=openni::STATUS_OK || !frame.isValid())
          throw std::runtime_error("depth read failed");
        if(frame.getFrameIndex()==last_index) {
          if(Clock::now()-last_fresh>std::chrono::seconds(3)) throw std::runtime_error("repeated depth frame");
          continue;
        }
        last_index=frame.getFrameIndex(); last_fresh=Clock::now();
        if(last_fresh<next) continue;
        write_state(summarize(frame,modes[selected].getFps()));
        next=last_fresh+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0/sample_hz));
      }
      unavailable("stopped");
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << ": " << openni::OpenNI::getExtendedError() << '\n';
    try { unavailable("capture_error"); } catch (...) {}
    result=70;
  }
  if(depth.isValid()) { depth.stop(); depth.destroy(); }
  if(device.isValid()) device.close();
  openni::OpenNI::shutdown();
  return result;
}
