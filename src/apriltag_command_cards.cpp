#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>

#include <AprilTags/TagDetector.h>

#include <AprilTags/Tag36h11.h>

#include <unordered_map>

#include <yaml-cpp/yaml.h>

int main(int argc, char **argv) {

  if (argc <= 1) {
    ROS_INFO("usage: apriltag_command_cards <command_cards.cfg>");
    return -1;
  }

  ros::init(argc, argv, "apriltag_command_cards",
            ros::init_options::NoSigintHandler);

  struct CommandCard {
    int id = 0;
    std::string name;
    double counter = 0.0;
    bool inhibition = true;
  };
  static std::vector<CommandCard> command_cards;
  static double timeout = 1.5;
  {
    ROS_INFO("command card description file %s", argv[1]);
    YAML::Node yaml = YAML::LoadFile(argv[1]);
    YAML::Node command_yaml = yaml["command_cards"];
    for (YAML::const_iterator it = command_yaml.begin();
         it != command_yaml.end(); ++it) {
      CommandCard command_card;
      command_card.id = it->first.as<int>();
      command_card.name = it->second.as<std::string>();
      command_cards.push_back(command_card);
    }
  }

  static ros::NodeHandle node;

  static ros::Publisher command_pub =
      node.advertise<std_msgs::String>("command_cards", 1);

  static image_transport::ImageTransport image_transport(node);

  static image_transport::Publisher image_pub =
      image_transport.advertise("command_card_detections", 1);

  auto *tag_codes = &AprilTags::tagCodes36h11;
  static auto tag_detector =
      std::make_shared<AprilTags::TagDetector>(*tag_codes);

  static ros::Time last_time = ros::Time::now();
  static image_transport::CameraSubscriber image_sub =
      image_transport.subscribeCamera(
          "/camera/rgb/image_rect_color", 1,
          (void (*)(const sensor_msgs::ImageConstPtr &,
                    const sensor_msgs::CameraInfoConstPtr &))[](
              const sensor_msgs::ImageConstPtr &msg,
              const sensor_msgs::CameraInfoConstPtr &camera) {
            ROS_INFO("image");
            ros::Time current_time = ros::Time::now();
            double delta_time = (current_time - last_time).toSec();
            last_time = current_time;
            cv_bridge::CvImagePtr cv_ptr;
            try {
              cv_ptr =
                  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            } catch (cv_bridge::Exception &e) {
              ROS_ERROR("cv_bridge exception: %s", e.what());
              return;
            }
            cv::Mat gray;
            cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

            std::vector<AprilTags::TagDetection> detections =
                tag_detector->extractTags(gray);
            ROS_INFO("%d tags detected", (int)detections.size());

            for (auto &detection : detections) {
              detection.draw(cv_ptr->image);
              for (auto &command_card : command_cards) {
                if (command_card.id == detection.id) {
                  cv::putText(
                      cv_ptr->image,
                      std::to_string((int)std::round(command_card.counter *
                                                     100 / timeout)),
                      cv::Point(detection.cxy.first, detection.cxy.second),
                      CV_FONT_HERSHEY_SIMPLEX, 0.8,
                      command_card.inhibition ? cv::Scalar(0, 0, 255)
                                              : cv::Scalar(0, 255, 0),
                      2, cv::LINE_AA);
                }
              }
            }

            for (auto &command_card : command_cards) {

              bool match = false;
              for (auto &detection : detections) {
                if (detection.id == command_card.id) {
                  match = true;
                }
              }

              if (match) {
                command_card.counter += delta_time;
                if (command_card.counter > timeout) {
                  command_card.counter = timeout;
                  if (!command_card.inhibition) {
                    command_card.inhibition = true;
                    ROS_INFO("command %i %s", command_card.id,
                             command_card.name.c_str());
                    std_msgs::String msg;
                    msg.data = command_card.name;
                    command_pub.publish(msg);
                  }
                }
              } else {
                command_card.counter -= delta_time;
                if (command_card.counter < 0) {
                  command_card.counter = 0;
                  command_card.inhibition = false;
                }
              }
            }

            image_pub.publish(cv_ptr->toImageMsg());
          });

  ROS_INFO("running");

  ros::spin();
}
