#include <algorithm>
#include <cctype>
#include <fstream>
#include <functional>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
class VioGpsControl : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr) override
  {
    node_.reset(new transport::Node());
    node_->Init(world->Name());
    control_sub_ = node_->Subscribe("~/vio/gps_control", &VioGpsControl::OnControl, this);
    status_pub_ = node_->Advertise<msgs::GzString>("~/vio/gps_control/status", 1, true);
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&VioGpsControl::OnUpdate, this));
    gzmsg << "[vio_gps_control] ready on ~/vio/gps_control\n";
  }

private:
  void OnControl(ConstGzStringPtr &message)
  {
    std::string command = message->data();
    std::transform(command.begin(), command.end(), command.begin(),
                   [](unsigned char value) { return static_cast<char>(std::tolower(value)); });
    const bool active = command == "on";
    if (!active && command != "off") {
      gzerr << "[vio_gps_control] expected 'on' or 'off', got '" << command << "'\n";
      return;
    }

    Apply(active, command);
  }

  void OnUpdate()
  {
    const bool requested_off = std::ifstream(kSentinel).good();
    if (requested_off != last_requested_off_) {
      last_requested_off_ = requested_off;
      Apply(!requested_off, requested_off ? "off" : "on");
    }
  }

  void Apply(bool active, const std::string &command)
  {
    unsigned int matched = 0;
    for (const auto &sensor : sensors::SensorManager::Instance()->GetSensors()) {
      if (sensor && sensor->Type() == "gps") {
        sensor->SetActive(active);
        ++matched;
      }
    }

    msgs::GzString status;
    status.set_data(command + ":gps_sensors=" + std::to_string(matched));
    status_pub_->Publish(status);
    gzmsg << "[vio_gps_control] " << status.data() << "\n";
  }

  transport::NodePtr node_;
  transport::SubscriberPtr control_sub_;
  transport::PublisherPtr status_pub_;
  event::ConnectionPtr update_connection_;
  bool last_requested_off_{false};
  static constexpr const char *kSentinel = "/tmp/project_f_vio_gps_off";
};

GZ_REGISTER_WORLD_PLUGIN(VioGpsControl)
}  // namespace gazebo
