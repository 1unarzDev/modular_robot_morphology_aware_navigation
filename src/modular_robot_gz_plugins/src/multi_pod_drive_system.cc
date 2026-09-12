#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/msgs/twist.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace modular_robot_gz_plugins
{
struct PodDrive
{
  std::string name;
  std::string leftJoint;
  std::string rightJoint;
  gz::sim::Entity leftEntity{gz::sim::kNullEntity};
  gz::sim::Entity rightEntity{gz::sim::kNullEntity};
  gz::transport::Node::Publisher odometryPublisher;
  double linear{0.0};
  double angular{0.0};
  double odomX{0.0};
  double odomY{0.0};
  double odomYaw{0.0};
  double previousLeft{0.0};
  double previousRight{0.0};
  bool encoderInitialized{false};
  std::chrono::steady_clock::duration lastOdomPublish{0};
  std::chrono::steady_clock::time_point lastCommand{};
};

class MultiPodDriveSystem final : public gz::sim::System,
                                  public gz::sim::ISystemConfigure,
                                  public gz::sim::ISystemPreUpdate,
                                  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    this->modelScope = gz::sim::scopedName(_entity, _ecm, "::", false);
    this->wheelSeparation = _sdf->Get<double>("wheel_separation", 0.14).first;
    this->wheelRadius = _sdf->Get<double>("wheel_radius", 0.055).first;
    this->maxWheelSpeed = _sdf->Get<double>("max_wheel_speed", 24.0).first;
    this->commandTimeout = std::chrono::duration<double>(
      _sdf->Get<double>("command_timeout", 0.3).first);
    if (!_sdf->HasElement("pod"))
      return;
    auto podElement = _sdf->FindElement("pod");
    while (podElement)
    {
      const auto podName = podElement->Get<std::string>();
      PodDrive pod;
      pod.name = podName;
      pod.leftJoint = this->modelScope + "::" + podName + "::" +
        podName + "_left_wheel_joint";
      pod.rightJoint = this->modelScope + "::" + podName + "::" +
        podName + "_right_wheel_joint";
      pod.odometryPublisher = this->node.Advertise<gz::msgs::Odometry>(
        "/model/" + podName + "/odometry");
      this->pods.emplace(podName, pod);
      this->node.Subscribe<gz::msgs::Twist>(
        "/model/" + podName + "/cmd_vel",
        std::function<void(const gz::msgs::Twist &)>([this, podName](
          const gz::msgs::Twist &_message)
        {
          std::lock_guard<std::mutex> guard(this->mutex);
          auto &drive = this->pods.at(podName);
          drive.linear = _message.linear().x();
          drive.angular = _message.angular().z();
          drive.lastCommand = std::chrono::steady_clock::now();
        }));
      podElement = podElement->GetNextElement("pod");
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    std::lock_guard<std::mutex> guard(this->mutex);
    const auto now = std::chrono::steady_clock::now();
    for (auto &[_, pod] : this->pods)
    {
      if (pod.leftEntity == gz::sim::kNullEntity)
        pod.leftEntity = this->JointByScopedName(pod.leftJoint, _ecm);
      if (pod.rightEntity == gz::sim::kNullEntity)
        pod.rightEntity = this->JointByScopedName(pod.rightJoint, _ecm);
      if (pod.leftEntity == gz::sim::kNullEntity ||
          pod.rightEntity == gz::sim::kNullEntity)
        continue;
      const auto active = now - pod.lastCommand <= this->commandTimeout;
      const auto linearCommand = active ? pod.linear : 0.0;
      const auto angularCommand = active ? pod.angular : 0.0;
      const auto halfYaw = angularCommand * this->wheelSeparation * 0.5;
      const auto left = std::clamp(
        (linearCommand - halfYaw) / this->wheelRadius,
        -this->maxWheelSpeed, this->maxWheelSpeed);
      const auto right = std::clamp(
        (linearCommand + halfYaw) / this->wheelRadius,
        -this->maxWheelSpeed, this->maxWheelSpeed);
      this->SetVelocity(pod.leftEntity, left, _ecm);
      this->SetVelocity(pod.rightEntity, right, _ecm);
      if (!_ecm.Component<gz::sim::components::JointPosition>(pod.leftEntity))
        _ecm.CreateComponent(
          pod.leftEntity, gz::sim::components::JointPosition());
      if (!_ecm.Component<gz::sim::components::JointPosition>(pod.rightEntity))
        _ecm.CreateComponent(
          pod.rightEntity, gz::sim::components::JointPosition());
    }
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused)
      return;
    std::lock_guard<std::mutex> guard(this->mutex);
    for (auto &[_, pod] : this->pods)
    {
      if (pod.leftEntity == gz::sim::kNullEntity ||
          pod.rightEntity == gz::sim::kNullEntity)
        continue;
      const auto left = _ecm.Component<gz::sim::components::JointPosition>(
        pod.leftEntity);
      const auto right = _ecm.Component<gz::sim::components::JointPosition>(
        pod.rightEntity);
      if (!left || !right || left->Data().empty() || right->Data().empty())
        continue;
      const auto leftAngle = left->Data()[0];
      const auto rightAngle = right->Data()[0];
      if (!pod.encoderInitialized)
      {
        pod.previousLeft = leftAngle;
        pod.previousRight = rightAngle;
        pod.encoderInitialized = true;
      }
      const auto leftDistance = (leftAngle - pod.previousLeft) * this->wheelRadius;
      const auto rightDistance = (rightAngle - pod.previousRight) * this->wheelRadius;
      pod.previousLeft = leftAngle;
      pod.previousRight = rightAngle;
      const auto distance = 0.5 * (leftDistance + rightDistance);
      const auto yawChange = (rightDistance - leftDistance) / this->wheelSeparation;
      pod.odomX += distance * std::cos(pod.odomYaw + 0.5 * yawChange);
      pod.odomY += distance * std::sin(pod.odomYaw + 0.5 * yawChange);
      pod.odomYaw += yawChange;
      if (_info.simTime - pod.lastOdomPublish < std::chrono::milliseconds(20))
        continue;
      pod.lastOdomPublish = _info.simTime;
      gz::msgs::Odometry message;
      message.mutable_pose()->mutable_position()->set_x(pod.odomX);
      message.mutable_pose()->mutable_position()->set_y(pod.odomY);
      message.mutable_pose()->mutable_orientation()->set_z(std::sin(pod.odomYaw * 0.5));
      message.mutable_pose()->mutable_orientation()->set_w(std::cos(pod.odomYaw * 0.5));
      pod.odometryPublisher.Publish(message);
    }
  }

private:
  static gz::sim::Entity JointByScopedName(
      const std::string &_name, const gz::sim::EntityComponentManager &_ecm)
  {
    const auto matches = gz::sim::entitiesFromScopedName(_name, _ecm);
    const auto found = std::find_if(matches.begin(), matches.end(), [&_ecm](auto entity) {
      return _ecm.EntityHasComponentType(entity, gz::sim::components::Joint::typeId);
    });
    return found == matches.end() ? gz::sim::kNullEntity : *found;
  }

  static void SetVelocity(gz::sim::Entity _joint, double _velocity,
                          gz::sim::EntityComponentManager &_ecm)
  {
    auto command = _ecm.Component<gz::sim::components::JointVelocityCmd>(_joint);
    if (command)
      command->SetData({_velocity}, [](const auto &, const auto &) {return false;});
    else
      _ecm.CreateComponent(
        _joint, gz::sim::components::JointVelocityCmd({_velocity}));
  }

  gz::transport::Node node;
  std::mutex mutex;
  std::string modelScope;
  std::unordered_map<std::string, PodDrive> pods;
  double wheelSeparation{0.14};
  double wheelRadius{0.055};
  double maxWheelSpeed{24.0};
  std::chrono::duration<double> commandTimeout{0.3};
};
}

GZ_ADD_PLUGIN(
  modular_robot_gz_plugins::MultiPodDriveSystem,
  gz::sim::System,
  modular_robot_gz_plugins::MultiPodDriveSystem::ISystemConfigure,
  modular_robot_gz_plugins::MultiPodDriveSystem::ISystemPreUpdate,
  modular_robot_gz_plugins::MultiPodDriveSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  modular_robot_gz_plugins::MultiPodDriveSystem,
  "modular_robot_gz_plugins::MultiPodDriveSystem")
