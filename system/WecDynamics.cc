// Copyright (C) 2026  maritime_ws
//
// WecDynamics -- accelerazioni hull + stato PTO. Vedi WecDynamics.hh.

#include "WecDynamics.hh"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport.hh>

#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForce.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

#include <sdf/Element.hh>

namespace gz
{
namespace sim
{
namespace wec
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

/// Primo ordine passa-basso: y += alpha * (x - y).
class LowPass
{
  public: void SetCutoff(double _cutoffHz, double _dt)
  {
    if (_cutoffHz <= 0.0 || _dt <= 0.0)
    {
      this->alpha = 1.0;
      return;
    }
    const double rc = 1.0 / (2.0 * kPi * _cutoffHz);
    this->alpha = _dt / (rc + _dt);
  }

  public: gz::math::Vector3d Apply(const gz::math::Vector3d &_x)
  {
    if (!this->primed)
    {
      this->y = _x;
      this->primed = true;
      return this->y;
    }
    this->y += this->alpha * (_x - this->y);
    return this->y;
  }

  public: void Reset()
  {
    this->primed = false;
    this->y = gz::math::Vector3d::Zero;
  }

  private: bool primed{false};
  private: double alpha{1.0};
  private: gz::math::Vector3d y{gz::math::Vector3d::Zero};
};

bool Require(const std::shared_ptr<const sdf::Element> &_sdf,
             const std::string &_key, std::string &_out)
{
  if (!_sdf || !_sdf->HasElement(_key))
  {
    gzerr << "WecDynamics: parametro SDF obbligatorio <" << _key
          << "> assente (generare il plugin da config/pipeline.json)\n";
    return false;
  }
  _out = _sdf->Get<std::string>(_key);
  return true;
}

bool Require(const std::shared_ptr<const sdf::Element> &_sdf,
             const std::string &_key, double &_out)
{
  if (!_sdf || !_sdf->HasElement(_key))
  {
    gzerr << "WecDynamics: parametro SDF obbligatorio <" << _key
          << "> assente (generare il plugin da config/pipeline.json)\n";
    return false;
  }
  _out = _sdf->Get<double>(_key);
  return true;
}

}  // namespace

class WecDynamicsPrivate
{
  public: bool Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm);

  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm);

  public: void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm);

  public: void Publish(const UpdateInfo &_info);

  // --- configurazione (da SDF, a sua volta da config/*.json) ---
  public: std::string baseLinkName;
  public: std::string ptoJointName;
  public: double ptoDamping{0.0};
  public: double ptoSpringK{0.0};
  public: double ptoSpringRef{0.0};
  public: double updateRate{0.0};
  public: double lpfCutoffHz{0.0};
  public: std::string accelTopic;
  public: std::string angAccelTopic;
  public: std::string ptoStateTopic;
  public: bool initialPositionIsMid{true};
  public: double initialPosition{0.0};

  // --- entita' ---
  public: Entity modelEntity{kNullEntity};
  public: Entity baseLink{kNullEntity};
  public: Entity ptoJoint{kNullEntity};

  // --- stato ---
  public: bool initialized{false};
  public: bool initialPoseApplied{false};
  public: double lastPubSimTime{-1.0};
  public: double energyJ{0.0};
  public: gz::math::Vector3d prevLinVel{gz::math::Vector3d::Zero};
  public: gz::math::Vector3d prevAngVel{gz::math::Vector3d::Zero};
  public: bool havePrevVel{false};
  public: double prevSimTime{0.0};
  public: LowPass linAccelLpf;
  public: LowPass angAccelLpf;
  public: gz::math::Vector3d linAccel{gz::math::Vector3d::Zero};
  public: gz::math::Vector3d angAccel{gz::math::Vector3d::Zero};
  public: double ptoPos{0.0};
  public: double ptoVel{0.0};
  public: double ptoForce{0.0};

  public: gz::transport::Node node;
  public: gz::transport::Node::Publisher pubAccel;
  public: gz::transport::Node::Publisher pubAngAccel;
  public: gz::transport::Node::Publisher pubPto;
};

//////////////////////////////////////////////////
bool WecDynamicsPrivate::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm)
{
  this->modelEntity = _entity;

  std::string initialPosRaw;
  if (!Require(_sdf, "base_link", this->baseLinkName) ||
      !Require(_sdf, "pto_joint", this->ptoJointName) ||
      !Require(_sdf, "pto_damping", this->ptoDamping) ||
      !Require(_sdf, "pto_spring_stiffness", this->ptoSpringK) ||
      !Require(_sdf, "pto_spring_reference", this->ptoSpringRef) ||
      !Require(_sdf, "update_rate", this->updateRate) ||
      !Require(_sdf, "lpf_cutoff_hz", this->lpfCutoffHz) ||
      !Require(_sdf, "accel_topic", this->accelTopic) ||
      !Require(_sdf, "angular_accel_topic", this->angAccelTopic) ||
      !Require(_sdf, "pto_state_topic", this->ptoStateTopic) ||
      !Require(_sdf, "initial_position", initialPosRaw))
  {
    return false;
  }

  if (initialPosRaw == "mid_stroke")
  {
    this->initialPositionIsMid = true;
  }
  else
  {
    this->initialPositionIsMid = false;
    this->initialPosition = std::stod(initialPosRaw);
  }

  Model model(_entity);
  this->baseLink = model.LinkByName(_ecm, this->baseLinkName);
  if (this->baseLink == kNullEntity)
  {
    gzerr << "WecDynamics: link base '" << this->baseLinkName << "' assente\n";
    return false;
  }
  this->ptoJoint = model.JointByName(_ecm, this->ptoJointName);
  if (this->ptoJoint == kNullEntity)
  {
    gzerr << "WecDynamics: giunto PTO '" << this->ptoJointName << "' assente\n";
    return false;
  }

  // forza la fisica a popolare le velocita' world (stessa tecnica di
  // Hydrodynamics: senza il componente la lettura e' vuota)
  if (!_ecm.Component<components::WorldLinearVelocity>(this->baseLink))
    _ecm.CreateComponent(this->baseLink, components::WorldLinearVelocity());
  if (!_ecm.Component<components::WorldAngularVelocity>(this->baseLink))
    _ecm.CreateComponent(this->baseLink, components::WorldAngularVelocity());

  if (!_ecm.Component<components::JointPosition>(this->ptoJoint))
    _ecm.CreateComponent(this->ptoJoint, components::JointPosition());
  if (!_ecm.Component<components::JointVelocity>(this->ptoJoint))
    _ecm.CreateComponent(this->ptoJoint, components::JointVelocity());
  if (!_ecm.Component<components::JointForce>(this->ptoJoint))
    _ecm.CreateComponent(this->ptoJoint, components::JointForce());

  this->pubAccel = this->node.Advertise<gz::msgs::Vector3d>(this->accelTopic);
  this->pubAngAccel =
      this->node.Advertise<gz::msgs::Vector3d>(this->angAccelTopic);
  this->pubPto = this->node.Advertise<gz::msgs::Float_V>(this->ptoStateTopic);
  if (!this->pubAccel || !this->pubAngAccel || !this->pubPto)
  {
    gzerr << "WecDynamics: advertise topic fallito\n";
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
void WecDynamicsPrivate::PreUpdate(const UpdateInfo &_info,
                                   EntityComponentManager &_ecm)
{
  if (this->modelEntity == kNullEntity)
    return;

  if (!this->initialPoseApplied)
  {
    double q0 = this->initialPosition;
    if (this->initialPositionIsMid)
    {
      // mid-stroke dai limiti del JointAxis (nessun letterale)
      auto axis = _ecm.Component<components::JointAxis>(this->ptoJoint);
      if (axis)
      {
        const double lo = axis->Data().Lower();
        const double hi = axis->Data().Upper();
        q0 = 0.5 * (lo + hi);
      }
    }
    auto reset = _ecm.Component<components::JointPositionReset>(this->ptoJoint);
    const std::vector<double> qv{q0};
    if (!reset)
      reset = _ecm.CreateComponent(this->ptoJoint,
                                   components::JointPositionReset(qv));
    else
      reset->SetData(qv, [](const std::vector<double> &,
                            const std::vector<double> &) { return false; });
    this->initialPoseApplied = true;
  }

  const double t = std::chrono::duration<double>(_info.simTime).count();
  const double dt = std::chrono::duration<double>(_info.dt).count();
  if (dt > 0.0)
  {
    this->linAccelLpf.SetCutoff(this->lpfCutoffHz, dt);
    this->angAccelLpf.SetCutoff(this->lpfCutoffHz, dt);
  }

  auto linComp = _ecm.Component<components::WorldLinearVelocity>(this->baseLink);
  auto angComp = _ecm.Component<components::WorldAngularVelocity>(this->baseLink);
  const gz::math::Vector3d linVel =
      linComp ? linComp->Data() : gz::math::Vector3d::Zero;
  const gz::math::Vector3d angVel =
      angComp ? angComp->Data() : gz::math::Vector3d::Zero;

  if (this->havePrevVel && dt > 0.0)
  {
    const gz::math::Vector3d a = (linVel - this->prevLinVel) / dt;
    const gz::math::Vector3d al = (angVel - this->prevAngVel) / dt;
    this->linAccel = this->linAccelLpf.Apply(a);
    this->angAccel = this->angAccelLpf.Apply(al);
  }
  this->prevLinVel = linVel;
  this->prevAngVel = angVel;
  this->prevSimTime = t;
  this->havePrevVel = true;

  auto posComp = _ecm.Component<components::JointPosition>(this->ptoJoint);
  auto velComp = _ecm.Component<components::JointVelocity>(this->ptoJoint);
  this->ptoPos =
      (posComp && !posComp->Data().empty()) ? posComp->Data()[0] : 0.0;
  this->ptoVel =
      (velComp && !velComp->Data().empty()) ? velComp->Data()[0] : 0.0;

  // forza assiale: se la fisica non la emette resta il modello PTO
  auto forceComp = _ecm.Component<components::JointForce>(this->ptoJoint);
  if (forceComp && !forceComp->Data().empty())
    this->ptoForce = forceComp->Data()[0];
  else
    this->ptoForce = -this->ptoDamping * this->ptoVel
                     - this->ptoSpringK * (this->ptoPos - this->ptoSpringRef);

  // potenza dissipata dallo smorzatore PTO (>= 0) e energia estratta
  const double power = this->ptoDamping * this->ptoVel * this->ptoVel;
  if (dt > 0.0)
    this->energyJ += power * dt;
}

//////////////////////////////////////////////////
void WecDynamicsPrivate::PostUpdate(const UpdateInfo &_info,
                                    const EntityComponentManager &)
{
  if (!_info.paused)
    this->Publish(_info);
}

//////////////////////////////////////////////////
void WecDynamicsPrivate::Publish(const UpdateInfo &_info)
{
  if (this->updateRate > 0.0)
  {
    const double t = std::chrono::duration<double>(_info.simTime).count();
    if (this->lastPubSimTime >= 0.0 &&
        (t - this->lastPubSimTime) < (1.0 / this->updateRate))
      return;
    this->lastPubSimTime = t;
  }

  {
    gz::msgs::Vector3d msg;
    msg.set_x(this->linAccel.X());
    msg.set_y(this->linAccel.Y());
    msg.set_z(this->linAccel.Z());
    this->pubAccel.Publish(msg);
  }
  {
    gz::msgs::Vector3d msg;
    msg.set_x(this->angAccel.X());
    msg.set_y(this->angAccel.Y());
    msg.set_z(this->angAccel.Z());
    this->pubAngAccel.Publish(msg);
  }
  {
    const double power = this->ptoDamping * this->ptoVel * this->ptoVel;
    gz::msgs::Float_V msg;
    msg.add_data(static_cast<float>(this->ptoPos));
    msg.add_data(static_cast<float>(this->ptoVel));
    msg.add_data(static_cast<float>(this->ptoForce));
    msg.add_data(static_cast<float>(power));
    msg.add_data(static_cast<float>(this->energyJ));
    this->pubPto.Publish(msg);
  }
}

//////////////////////////////////////////////////
WecDynamics::WecDynamics() : dataPtr(new WecDynamicsPrivate) {}
WecDynamics::~WecDynamics() = default;

//////////////////////////////////////////////////
void WecDynamics::Configure(const Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            EntityComponentManager &_ecm,
                            EventManager &)
{
  this->dataPtr->initialized =
      this->dataPtr->Configure(_entity, _sdf, _ecm);
}

//////////////////////////////////////////////////
void WecDynamics::PreUpdate(const UpdateInfo &_info,
                            EntityComponentManager &_ecm)
{
  if (this->dataPtr->initialized)
    this->dataPtr->PreUpdate(_info, _ecm);
}

//////////////////////////////////////////////////
void WecDynamics::PostUpdate(const UpdateInfo &_info,
                             const EntityComponentManager &_ecm)
{
  if (this->dataPtr->initialized)
    this->dataPtr->PostUpdate(_info, _ecm);
}

}  // namespace wec
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(gz::sim::wec::WecDynamics,
              gz::sim::System,
              gz::sim::wec::WecDynamics::ISystemConfigure,
              gz::sim::wec::WecDynamics::ISystemPreUpdate,
              gz::sim::wec::WecDynamics::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::wec::WecDynamics, "wec::WecDynamics")
