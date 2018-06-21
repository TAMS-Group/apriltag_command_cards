#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
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

  static ros::NodeHandle node;

  struct CommandCard {
    int id = 0;
    std::string name;
    double counter = 0.0;
    bool inhibition = true;
    ros::Publisher command_pub, visibility_pub, direction_pub;
  };
  static std::vector<CommandCard> command_cards;
  static double timeout = 1;
  {
    ROS_INFO("command card description file %s", argv[1]);
    YAML::Node yaml = YAML::LoadFile(argv[1]);
    if (auto p = yaml["timeout"]) {
      timeout = p.as<double>();
    }
    YAML::Node command_yaml = yaml["command_cards"];
    for (YAML::const_iterator it = command_yaml.begin();
         it != command_yaml.end(); ++it) {
      CommandCard command_card;
      command_card.id = it->first.as<int>();
      command_card.name = it->second.as<std::string>();
      std::string topic_name = command_card.name;
      for (auto &c : topic_name) {
        if (!std::isalnum(c)) {
          c = '_';
        }
      }
      command_card.command_pub = node.advertise<std_msgs::Empty>(
          "/command_cards/cards/" + topic_name + "/command", 1);
      command_card.visibility_pub = node.advertise<std_msgs::Bool>(
          "/command_cards/cards/" + topic_name + "/visible", 1);
      command_card.direction_pub = node.advertise<geometry_msgs::Vector3>(
          "/command_cards/cards/" + topic_name + "/direction", 1);
      command_cards.push_back(command_card);
    }
  }

  static ros::Publisher command_pub =
      node.advertise<std_msgs::String>("/command_cards/commands", 1);

  static image_transport::ImageTransport image_transport(node);

  static image_transport::Publisher image_pub =
      image_transport.advertise("/command_cards/image", 1);

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
            // ROS_INFO("image");
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
            // ROS_INFO("%d tags detected", (int)detections.size());

            auto putText = [&](const std::string &text, int x, int y,
                               const cv::Scalar &color) {
              cv::putText(cv_ptr->image, text, cv::Point(x, y),
                          CV_FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 8,
                          cv::LINE_AA);
              cv::putText(cv_ptr->image, text, cv::Point(x, y),
                          CV_FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv::LINE_AA);
            };

            for (auto &detection : detections) {
              detection.draw(cv_ptr->image);
              for (auto &command_card : command_cards) {
                if (command_card.id == detection.id) {
                  putText(command_card.name, detection.cxy.first,
                          detection.cxy.second - 20,
                          command_card.inhibition ? cv::Scalar(0, 0, 255)
                                                  : cv::Scalar(0, 255, 0));
                  putText(std::to_string((int)std::round(command_card.counter *
                                                         100 / timeout)),
                          detection.cxy.first, detection.cxy.second + 40,
                          command_card.inhibition ? cv::Scalar(0, 0, 255)
                                                  : cv::Scalar(0, 255, 0));
                }
              }
            }

            double fx = camera->P[0];
            double fy = camera->P[5];
            double px = camera->P[2];
            double py = camera->P[6];
            // ROS_INFO("%f %f %f %f", fx, fy, px, py);

            static std::string command_vis_text;
            static ros::Time command_vis_time;

            for (auto &command_card : command_cards) {

              bool match = false;
              double x = 0;
              double y = 0;
              double z = 0;
              for (auto &detection : detections) {
                if (detection.id == command_card.id) {
                  match = true;
                  x = +((detection.cxy.first - px) / fx);
                  y = -((detection.cxy.second - py) / fy);
                  z = 1;
                  double f = 1.0 / (x * x + y * y + z * z);
                  x *= f;
                  y *= f;
                  z *= f;
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
                    {
                      std_msgs::String msg;
                      msg.data = command_card.name;
                      command_pub.publish(msg);
                    }
                    {
                      std_msgs::Empty msg;
                      command_card.command_pub.publish(msg);
                    }
                    command_vis_text = command_card.name;
                    command_vis_time = current_time;
                  }
                }
              } else {
                command_card.counter -= delta_time;
                if (command_card.counter < 0) {
                  command_card.counter = 0;
                  command_card.inhibition = false;
                }
              }

              {
                std_msgs::Bool msg;
                msg.data = match;
                command_card.visibility_pub.publish(msg);
              }

              {
                geometry_msgs::Vector3 msg;
                msg.x = x;
                msg.y = y;
                msg.z = z;
                command_card.direction_pub.publish(msg);
              }
            }

            if (!command_vis_text.empty()) {
              putText(command_vis_text, 20, 60,
                      current_time == command_vis_time
                          ? cv::Scalar(0, 255, 255)
                          : cv::Scalar(255, 0, 255));
              if ((current_time - command_vis_time).toSec() > 1) {
                command_vis_text.clear();
              }
            }

            image_pub.publish(cv_ptr->toImageMsg());
          });

  ROS_INFO("running");

  ros::spin();
}
