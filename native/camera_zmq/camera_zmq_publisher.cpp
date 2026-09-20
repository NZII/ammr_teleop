#include <libobsensor/ObSensor.hpp>
#include <librealsense2/rs.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <zmq.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic_bool running{true};

struct Options {
  std::string backend;
  std::string camera_name;
  std::string serial;
  std::string usb_port;
  std::string bind;
  int width = 640;
  int height = 480;
  int fps = 30;
  int jpeg_quality = 92;
};

void on_signal(int) { running.store(false); }

uint64_t monotonic_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

std::string value_after(int &index, int argc, char **argv) {
  if (++index >= argc) throw std::invalid_argument("missing option value");
  return argv[index];
}

Options parse_options(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--backend") options.backend = value_after(i, argc, argv);
    else if (arg == "--camera-name") options.camera_name = value_after(i, argc, argv);
    else if (arg == "--serial") options.serial = value_after(i, argc, argv);
    else if (arg == "--usb-port") options.usb_port = value_after(i, argc, argv);
    else if (arg == "--bind") options.bind = value_after(i, argc, argv);
    else if (arg == "--width") options.width = std::stoi(value_after(i, argc, argv));
    else if (arg == "--height") options.height = std::stoi(value_after(i, argc, argv));
    else if (arg == "--fps") options.fps = std::stoi(value_after(i, argc, argv));
    else if (arg == "--jpeg-quality") options.jpeg_quality = std::stoi(value_after(i, argc, argv));
    else if (arg == "--help") {
      std::cout << "Usage: ammr_camera_zmq_publisher --backend realsense|orbbec "
                   "--camera-name NAME --serial SERIAL --bind tcp://0.0.0.0:PORT\n";
      std::exit(0);
    } else throw std::invalid_argument("unknown option: " + arg);
  }
  if (options.backend != "realsense" && options.backend != "orbbec")
    throw std::invalid_argument("--backend must be realsense or orbbec");
  if (options.camera_name != "head" && options.camera_name != "left_wrist" &&
      options.camera_name != "right_wrist")
    throw std::invalid_argument("invalid --camera-name");
  if (options.serial.empty() || options.bind.empty())
    throw std::invalid_argument("--serial and --bind are required");
  if (options.width <= 0 || options.height <= 0 || options.fps <= 0 ||
      options.jpeg_quality < 1 || options.jpeg_quality > 100)
    throw std::invalid_argument("invalid image dimensions, fps, or JPEG quality");
  return options;
}

class Publisher {
 public:
  explicit Publisher(const std::string &endpoint) {
    context_ = zmq_ctx_new();
    socket_ = zmq_socket(context_, ZMQ_PUB);
    if (!context_ || !socket_) throw std::runtime_error("failed to create ZMQ PUB socket");
    int hwm = 2, linger = 0, immediate = 1;
    zmq_setsockopt(socket_, ZMQ_SNDHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(socket_, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(socket_, ZMQ_IMMEDIATE, &immediate, sizeof(immediate));
    if (zmq_bind(socket_, endpoint.c_str()) != 0)
      throw std::runtime_error("ZMQ bind failed: " + std::string(zmq_strerror(zmq_errno())));
  }

  ~Publisher() {
    if (socket_) zmq_close(socket_);
    if (context_) zmq_ctx_term(context_);
  }

  bool send(const Options &options, uint64_t sequence, uint64_t source_timestamp_ns,
            uint64_t capture_monotonic_ns, const uint8_t *jpeg, size_t jpeg_size,
            int width, int height, int advertised_quality) {
    std::ostringstream metadata;
    metadata << "{\"schema_version\":1,\"camera_name\":\"" << options.camera_name
             << "\",\"sequence\":" << sequence
             << ",\"source_timestamp_ns\":" << source_timestamp_ns
             << ",\"capture_monotonic_ns\":" << capture_monotonic_ns
             << ",\"bundle_sequence\":null,\"width\":" << width
             << ",\"height\":" << height
             << ",\"encoding\":\"bgr8\",\"codec\":\"jpeg\",\"jpeg_quality\":"
             << advertised_quality << "}";
    const std::string header = metadata.str();
    const int first = zmq_send(socket_, header.data(), header.size(), ZMQ_SNDMORE | ZMQ_DONTWAIT);
    if (first < 0) {
      if (zmq_errno() == EAGAIN) return false;
      throw std::runtime_error("ZMQ metadata send failed");
    }
    const int second = zmq_send(socket_, jpeg, jpeg_size, ZMQ_DONTWAIT);
    if (second < 0) {
      if (zmq_errno() == EAGAIN) return false;
      throw std::runtime_error("ZMQ image send failed");
    }
    return true;
  }

 private:
  void *context_ = nullptr;
  void *socket_ = nullptr;
};

std::shared_ptr<ob::Device> find_orbbec(const std::string &serial,
                                       const std::string &usb_port) {
  ob::Context context;
  auto list = context.queryDeviceList();
  if (list->deviceCount() == 0)
    throw std::runtime_error("no Orbbec devices found");
  for (uint32_t i = 0; i < list->deviceCount(); ++i)
    std::cout << "Orbbec candidate uid=" << list->uid(i)
              << " connection=" << list->connectionType(i) << std::endl;
  // DaBai DC1 is an OpenNI device and does not expose its serial until it is
  // opened. Select by the stable physical USB topology first, then verify SN.
  auto device = usb_port.empty() ? list->getDeviceBySN(serial.c_str())
                                 : list->getDeviceByUid(usb_port.c_str());
  if (!device) throw std::runtime_error("Orbbec device not found: " + serial);
  const std::string actual = device->getDeviceInfo()->serialNumber();
  if (actual != serial)
    throw std::runtime_error("Orbbec USB port " + usb_port + " has serial " +
                             actual + ", expected " + serial);
  return device;
}

void run_orbbec(const Options &options, Publisher &publisher) {
  auto device = find_orbbec(options.serial, options.usb_port);
  ob::Pipeline pipeline(device);
  auto profiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
  std::shared_ptr<ob::VideoStreamProfile> profile;
  try {
    profile = profiles->getVideoStreamProfile(
        options.width, options.height, OB_FORMAT_MJPG, options.fps);
  } catch (const ob::Error &) {
    profile = profiles->getVideoStreamProfile(
        options.width, options.height, OB_FORMAT_RGB, options.fps);
  }
  auto config = std::make_shared<ob::Config>();
  config->enableStream(profile);
  pipeline.start(config);
  std::cout << "Orbbec " << options.serial << " " << profile->width() << "x"
            << profile->height() << "@" << profile->fps() << " format="
            << static_cast<int>(profile->format()) << std::endl;
  while (running.load()) {
    auto frames = pipeline.waitForFrames(1000);
    if (!frames) continue;
    auto color = frames->colorFrame();
    if (!color) continue;
    const uint64_t captured = monotonic_ns();
    const uint64_t source_ns = color->timeStampUs() * 1000ULL;
    if (color->format() == OB_FORMAT_MJPG) {
      publisher.send(options, color->index(), source_ns, captured,
                     static_cast<const uint8_t *>(color->data()), color->dataSize(),
                     profile->width(), profile->height(), 0);
      continue;
    }
    cv::Mat rgb(profile->height(), profile->width(), CV_8UC3, color->data());
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    std::vector<uint8_t> jpeg;
    cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, options.jpeg_quality});
    publisher.send(options, color->index(), source_ns, captured, jpeg.data(), jpeg.size(),
                   profile->width(), profile->height(), options.jpeg_quality);
  }
  pipeline.stop();
}

void run_realsense(const Options &options, Publisher &publisher) {
  rs2::pipeline pipeline;
  rs2::config config;
  config.enable_device(options.serial);
  config.enable_stream(RS2_STREAM_COLOR, options.width, options.height,
                       RS2_FORMAT_BGR8, options.fps);
  auto profile = pipeline.start(config);
  auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
  const auto intrinsics = stream.get_intrinsics();
  std::cout << "RealSense " << options.serial << " " << intrinsics.width << "x"
            << intrinsics.height << " fx=" << intrinsics.fx << " fy=" << intrinsics.fy
            << " cx=" << intrinsics.ppx << " cy=" << intrinsics.ppy << std::endl;
  while (running.load()) {
    rs2::frameset frames;
    if (!pipeline.poll_for_frames(&frames)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    auto color = frames.get_color_frame();
    if (!color) continue;
    const uint64_t captured = monotonic_ns();
    const uint64_t source_ns = static_cast<uint64_t>(color.get_timestamp() * 1.0e6);
    cv::Mat bgr(options.height, options.width, CV_8UC3,
                const_cast<void *>(color.get_data()), cv::Mat::AUTO_STEP);
    std::vector<uint8_t> jpeg;
    cv::imencode(".jpg", bgr, jpeg, {cv::IMWRITE_JPEG_QUALITY, options.jpeg_quality});
    publisher.send(options, color.get_frame_number(), source_ns, captured,
                   jpeg.data(), jpeg.size(), options.width, options.height,
                   options.jpeg_quality);
  }
  pipeline.stop();
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const Options options = parse_options(argc, argv);
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    Publisher publisher(options.bind);
    std::cout << "Publishing " << options.camera_name << " on " << options.bind << std::endl;
    if (options.backend == "orbbec") run_orbbec(options, publisher);
    else run_realsense(options, publisher);
    return 0;
  } catch (const ob::Error &error) {
    std::cerr << "OrbbecSDK error: " << error.getMessage() << std::endl;
  } catch (const rs2::error &error) {
    std::cerr << "librealsense error: " << error.what() << std::endl;
  } catch (const std::exception &error) {
    std::cerr << "camera publisher error: " << error.what() << std::endl;
  }
  return 1;
}
