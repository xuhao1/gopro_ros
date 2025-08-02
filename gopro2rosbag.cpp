//
// Created by bjoshi on 10/29/20.
//

#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Header.h>

#include <experimental/filesystem>
#include <iostream>

#include "ImuExtractor.h"
#include "VideoExtractor.h"
#include "color_codes.h"

namespace po = boost::program_options;

using namespace std;
namespace fs = std::experimental::filesystem;

int main(int argc, char* argv[]) {
  spdlog::set_level(spdlog::level::info);
  spdlog::info("Welcome to GoPro to rosbag converter");
  // Use boost program options
  string gopro_video = "";
  string gopro_folder = "";
  string rosbag = "";
  double scaling = 0.5;
  bool compress_images = true;
  bool grayscale = false;
  bool display_images = false;
  bool multiple_files = false;
  bool is_gopro_folder = false;
  po::options_description p("gopro2rosbag options");
  p.add_options()
    ("gopro-video", po::value<string>(&gopro_video), "GoPro video file")
    ("gopro-folder", po::value<string>(&gopro_folder), "GoPro video folder")
    ("rosbag", po::value<string>(&rosbag), "rosbag file")
    ("scale", po::value<double>(&scaling), "scaling factor")
    ("compressed_image_format", po::value<bool>(&compress_images), "compress images")
    ("grayscale", po::value<bool>(&grayscale), "grayscale images")
    ("display_images", po::value<bool>(&display_images), "display images")
    ("multiple_files", po::value<bool>(&multiple_files), "multiple files");
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, p), vm);
  po::notify(vm);

  if (gopro_video.empty() && gopro_folder.empty()) {
    spdlog::error("Please specify the gopro video or folder");
    exit(-1);
  }
  if (gopro_video.empty() && !gopro_folder.empty()) {
    spdlog::info("Processing GoPro folder");
    is_gopro_folder = true;
  }
  if (rosbag.empty()) {
    //Replace .MP4 with .bag for gopro video
    if (!gopro_video.empty()) {
      rosbag = gopro_video;
      rosbag.replace(rosbag.find(".MP4"), 4, ".bag");
    } else {
      rosbag = gopro_folder + ".bag";
    }
    spdlog::info("rosbag file not specified. Using {}", rosbag);
  }
  spdlog::info("GoPro Video: {}\n Output: {}\n Scaling: {}\n Compress Images: {}\n Grayscale: {}\n Display Images: {}\n Multiple Files: {}\n",
               gopro_video, rosbag, scaling, compress_images, grayscale, display_images, multiple_files);

  rosbag::Bag bag;
  bag.open(rosbag, rosbag::bagmode::Write);

  vector<fs::path> video_files;

  if (is_gopro_folder && multiple_files) {
    std::copy(fs::directory_iterator(gopro_folder),
              fs::directory_iterator(),
              std::back_inserter(video_files));
    std::sort(video_files.begin(), video_files.end());

  } else {
    video_files.push_back(fs::path(gopro_video));
  }

  auto end = std::remove_if(video_files.begin(), video_files.end(), [](const fs::path& p) {
    return p.extension() != ".MP4" || fs::is_directory(p);
  });
  video_files.erase(end, video_files.end());

  vector<uint64_t> start_stamps;
  vector<uint32_t> samples;
  std::deque<AcclMeasurement> accl_queue;
  std::deque<GyroMeasurement> gyro_queue;
  std::deque<MagMeasurement> magnetometer_queue;

  vector<uint64_t> image_stamps;

  bool has_magnetic_field_readings = false;
  for (uint32_t i = 0; i < video_files.size(); i++) {
    image_stamps.clear();

    fs::path file = video_files[i];
    GoProImuExtractor imu_extractor(file.string());
    GoProVideoExtractor video_extractor(file.string(), scaling, true);

    // if (i == 0 && imu_extractor.getNumofSamples(STR2FOURCC("MAGN"))) {
    //   has_magnetic_field_readings = true;
    // }

    imu_extractor.getPayloadStamps(STR2FOURCC("ACCL"), start_stamps, samples);
    spdlog::info("[ACCL] Payloads: {} Start stamp: {} End stamp: {} Total Samples: {}",
                 start_stamps.size(), start_stamps[0], start_stamps[samples.size() - 1], samples.at(samples.size() - 1));

    imu_extractor.getPayloadStamps(STR2FOURCC("GYRO"), start_stamps, samples);
    spdlog::info("[GYRO] Payloads: {} Start stamp: {} End stamp: {} Total Samples: {}",
                 start_stamps.size(), start_stamps[0], start_stamps[samples.size() - 1], samples.at(samples.size() - 1));
    imu_extractor.getPayloadStamps(STR2FOURCC("CORI"), start_stamps, samples);
    spdlog::info("[Image] Payloads: {} Start stamp: {} End stamp: {} Total Samples: {}",
                 start_stamps.size(), start_stamps[0], start_stamps[samples.size() - 1], samples.at(samples.size() - 1));
    if (has_magnetic_field_readings) {
      imu_extractor.getPayloadStamps(STR2FOURCC("MAGN"), start_stamps, samples);
      ROS_INFO_STREAM("[MAGN] Payloads: " << start_stamps.size()
                                          << " Start stamp: " << start_stamps[0]
                                          << " End stamp: " << start_stamps[samples.size() - 1]
                                          << " Total Samples: " << samples.at(samples.size() - 1));
    }

    uint64_t accl_end_stamp = 0, gyro_end_stamp = 0;
    uint64_t video_end_stamp = 0;
    uint64_t magnetometer_end_stamp = 0;

    if (i < video_files.size() - 1) {
      GoProImuExtractor imu_extractor_next(video_files[i + 1].string());
      accl_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("ACCL"), 0);
      gyro_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("GYRO"), 0);
      video_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("CORI"), 0);
      if (has_magnetic_field_readings) {
        magnetometer_end_stamp = imu_extractor_next.getPayloadStartStamp(STR2FOURCC("MAGN"), 0);
      }
    }

    imu_extractor.readImuData(accl_queue, gyro_queue, accl_end_stamp, gyro_end_stamp);
    imu_extractor.readMagnetometerData(magnetometer_queue, magnetometer_end_stamp);

    uint32_t gpmf_frame_count = imu_extractor.getImageCount();
    uint32_t ffmpeg_frame_count = video_extractor.getFrameCount();
    if (gpmf_frame_count != ffmpeg_frame_count) {
      spdlog::error("Video and metadata frame count do not match");
      exit(-1);
    }

    uint64_t gpmf_video_time = imu_extractor.getVideoCreationTime();
    uint64_t ffmpeg_video_time = video_extractor.getVideoCreationTime();

    if (ffmpeg_video_time != gpmf_video_time) {
      spdlog::error("Video creation time does not match");
      exit(-1);
    }

    imu_extractor.getImageStamps(image_stamps, video_end_stamp);
    if (i != video_files.size() - 1 && image_stamps.size() != ffmpeg_frame_count) {
      // ROS_FATAL_STREAM(image_stamps.size() << " vs " << ffmpeg_frame_count);
      spdlog::error("ffmpeg and gpmf frame count does not match.");
      exit(-1);
    }

    video_extractor.writeVideo(
        bag, "/gopro/image_raw", image_stamps, grayscale, compress_images, display_images);
  }

  spdlog::info("[ACCL] Payloads: {}", accl_queue.size());
  spdlog::info("[GYRO] Payloads: {}", gyro_queue.size());

  assert(accl_queue.size() == gyro_queue.size());

  double previous = accl_queue.front().timestamp_ * 1e-9;

  while (!accl_queue.empty() && !gyro_queue.empty()) {
    AcclMeasurement accl = accl_queue.front();
    GyroMeasurement gyro = gyro_queue.front();
    int64_t diff = accl.timestamp_ - gyro.timestamp_;

    Timestamp stamp;
    if (abs(diff) > 100000) {
      // I will need to handle this case more carefully
      spdlog::warn("{} ns difference between gyro and accl", diff);
      stamp = (Timestamp)(((double)accl.timestamp_ + (double)gyro.timestamp_) / 2.0);
    } else {
      stamp = accl.timestamp_;
    }

    double current = stamp * 1e-9;
    // assert(abs(current - previous) < 0.001);
    previous = current;
    uint32_t secs = stamp * 1e-9;
    uint32_t n_secs = stamp % 1000000000;
    ros::Time ros_time(secs, n_secs);
    sensor_msgs::Imu imu_msg;
    std_msgs::Header header;
    header.stamp = ros_time;
    header.frame_id = "body";
    imu_msg.header = header;
    imu_msg.linear_acceleration.x = accl.data_.x();
    imu_msg.linear_acceleration.y = accl.data_.y();
    imu_msg.linear_acceleration.z = accl.data_.z();
    imu_msg.angular_velocity.x = gyro.data_.x();
    imu_msg.angular_velocity.y = gyro.data_.y();
    imu_msg.angular_velocity.z = gyro.data_.z();

    bag.write("/gopro/imu", ros_time, imu_msg);

    accl_queue.pop_front();
    gyro_queue.pop_front();
  }

  while (!magnetometer_queue.empty()) {
    MagMeasurement mag = magnetometer_queue.front();
    Timestamp stamp = mag.timestamp_;

    uint32_t secs = stamp * 1e-9;
    uint32_t n_secs = stamp % 1000000000;
    ros::Time ros_time(secs, n_secs);
    sensor_msgs::MagneticField magnetic_field_msg;
    std_msgs::Header header;
    header.stamp = ros_time;
    header.frame_id = "body";
    magnetic_field_msg.header = header;
    magnetic_field_msg.magnetic_field.x = mag.magfield_.x();
    magnetic_field_msg.magnetic_field.y = mag.magfield_.y();
    magnetic_field_msg.magnetic_field.z = mag.magfield_.z();

    bag.write("/gopro/magnetic_field", ros_time, magnetic_field_msg);

    magnetometer_queue.pop_front();
  }
}
