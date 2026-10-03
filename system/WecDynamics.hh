// Copyright (C) 2026  maritime_ws
//
// WecDynamics -- telemetria dinamica del dispositivo WEC.
//
// Pubblica accelerazioni lineari/angolari dell'hull e stato del giunto PTO
// (posa, velocita', forza, potenza, energia). Tutti i parametri fisici e i
// nomi dei topic arrivano dal blocco SDF del plugin, a sua volta generato
// da config/pipeline.json (nessun letterale nel sorgente).

#ifndef WEC_TOOLS_WEC_DYNAMICS_HH_
#define WEC_TOOLS_WEC_DYNAMICS_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
namespace wec
{

class WecDynamicsPrivate;

/// \brief Publisher di dinamica/telemetria lato server.
///
/// Uso in SDF (generato da build_simulation.py da config/pipeline.json):
/// \code
/// <plugin filename="WecDynamics" name="wec::WecDynamics">
///   <base_link>hull</base_link>
///   <pto_joint>slider</pto_joint>
///   <pto_damping>200</pto_damping>
///   <pto_spring_stiffness>10000</pto_spring_stiffness>
///   <pto_spring_reference>-12</pto_spring_reference>
///   <initial_position>mid_stroke</initial_position>  <!-- oppure un valore -->
///   <update_rate>50</update_rate>
///   <lpf_cutoff_hz>2</lpf_cutoff_hz>   <!-- da wave_band, non letterale -->
///   <accel_topic>...</accel_topic>
///   <angular_accel_topic>...</angular_accel_topic>
///   <pto_state_topic>...</pto_state_topic>
/// </plugin>
/// \endcode
class WecDynamics
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemPostUpdate
{
  public: WecDynamics();
  public: ~WecDynamics() override;

  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) final;

  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override;

  public: void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) override;

  private: std::unique_ptr<WecDynamicsPrivate> dataPtr;
};

}  // namespace wec
}  // namespace sim
}  // namespace gz

#endif
