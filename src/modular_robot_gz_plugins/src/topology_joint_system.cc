#include <algorithm>
#include <chrono>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace modular_robot_gz_plugins
{
class TopologyJointSystem final : public gz::sim::System,
                                  public gz::sim::ISystemConfigure,
                                  public gz::sim::ISystemPreUpdate,
                                  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    this->parentLink = _sdf->Get<std::string>("parent_link", "core::base_link").first;
    this->trackedLinks.push_back({"core", this->parentLink});
    const auto topic = _sdf->Get<std::string>("command_topic", "/topology_joint/command").first;
    if (_sdf->HasElement("initial_child"))
    {
      sdf::ElementConstPtr child = _sdf->FindElement("initial_child");
      while (child)
      {
        const auto childName = child->Get<std::string>();
        this->pending.emplace_back("attach", childName);
        this->trackedLinks.push_back({childName.substr(0, childName.find("::")), childName});
        child = child->GetNextElement("initial_child");
      }
    }
    this->node.Subscribe(topic, &TopologyJointSystem::OnCommand, this);
    this->statePublisher = this->node.Advertise<gz::msgs::StringMsg>("/topology_joint/state");
    this->posePublisher = this->node.Advertise<gz::msgs::StringMsg>("/topology_joint/pose");
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || _info.simTime - this->lastPosePublish < std::chrono::milliseconds(50))
      return;
    this->lastPosePublish = _info.simTime;
    for (const auto &[moduleName, linkName] : this->trackedLinks)
    {
      const auto entity = LinkByScopedName(linkName, _ecm);
      if (entity == gz::sim::kNullEntity)
        continue;
      const auto pose = gz::sim::worldPose(entity, _ecm);
      std::ostringstream stream;
      stream.precision(12);
      stream << "pose|" << moduleName << '|' << pose.Pos().X() << '|'
             << pose.Pos().Y() << '|' << pose.Rot().Euler().Z();
      this->PublishPose(stream.str());
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    std::vector<std::pair<std::string, std::string>> commands;
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      commands.swap(this->pending);
    }
    for (const auto &[operation, childName] : commands)
    {
      if (operation == "detach")
        this->Detach(childName, _ecm);
      else if (operation == "attach")
        this->Attach(childName, _ecm);
    }
  }

private:
  void OnCommand(const gz::msgs::StringMsg &_message)
  {
    const auto separator = _message.data().find('|');
    if (separator == std::string::npos)
      return;
    const auto operation = _message.data().substr(0, separator);
    const auto pod = _message.data().substr(separator + 1);
    if (operation != "attach" && operation != "detach")
      return;
    std::lock_guard<std::mutex> guard(this->mutex);
    this->pending.emplace_back(operation, pod + "::base_link");
  }

  static gz::sim::Entity LinkByScopedName(
      const std::string &_name, const gz::sim::EntityComponentManager &_ecm)
  {
    const auto matches = gz::sim::entitiesFromScopedName(_name, _ecm);
    const auto found = std::find_if(matches.begin(), matches.end(), [&_ecm](auto entity) {
      return _ecm.EntityHasComponentType(entity, gz::sim::components::Link::typeId);
    });
    return found == matches.end() ? gz::sim::kNullEntity : *found;
  }

  void Attach(const std::string &_childName, gz::sim::EntityComponentManager &_ecm)
  {
    if (this->joints.count(_childName) != 0)
      return;
    const auto parent = LinkByScopedName(this->parentLink, _ecm);
    const auto child = LinkByScopedName(_childName, _ecm);
    if (parent == gz::sim::kNullEntity || child == gz::sim::kNullEntity)
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      this->pending.emplace_back("attach", _childName);
      return;
    }
    const auto joint = _ecm.CreateEntity();
    _ecm.CreateComponent(joint, gz::sim::components::DetachableJoint({parent, child, "fixed"}));
    this->joints.emplace(_childName, joint);
    this->PublishState("attached|" + _childName);
  }

  void Detach(const std::string &_childName, gz::sim::EntityComponentManager &_ecm)
  {
    const auto found = this->joints.find(_childName);
    if (found == this->joints.end())
      return;
    _ecm.RequestRemoveEntity(found->second);
    this->joints.erase(found);
    this->PublishState("detached|" + _childName);
  }

  void PublishState(const std::string &_state)
  {
    gz::msgs::StringMsg message;
    message.set_data(_state);
    this->statePublisher.Publish(message);
  }

  void PublishPose(const std::string &_state)
  {
    gz::msgs::StringMsg message;
    message.set_data(_state);
    this->posePublisher.Publish(message);
  }

  gz::transport::Node node;
  gz::transport::Node::Publisher statePublisher;
  gz::transport::Node::Publisher posePublisher;
  std::string parentLink;
  std::chrono::steady_clock::duration lastPosePublish{0};
  std::mutex mutex;
  std::vector<std::pair<std::string, std::string>> pending;
  std::unordered_map<std::string, gz::sim::Entity> joints;
  std::vector<std::pair<std::string, std::string>> trackedLinks;
};
}

GZ_ADD_PLUGIN(
  modular_robot_gz_plugins::TopologyJointSystem,
  gz::sim::System,
  modular_robot_gz_plugins::TopologyJointSystem::ISystemConfigure,
  modular_robot_gz_plugins::TopologyJointSystem::ISystemPreUpdate,
  modular_robot_gz_plugins::TopologyJointSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  modular_robot_gz_plugins::TopologyJointSystem,
  "modular_robot_gz_plugins::TopologyJointSystem")
