#ifndef GZ_MOORING_MOORING_HH_
#define GZ_MOORING_MOORING_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
namespace mooring
{

class MooringPrivate;

/// \brief Sistema di ormeggio MoorDyn v2 accoppiato a gz-sim.
///
/// Applica le forze di ormeggio ai fairlead definiti in SDF e accoppia la
/// cinematica d'onda ai nodi dei cavi (WaveKin = 1).
///
/// Uso in SDF:
/// \code
/// <plugin filename="MooringSystem" name="gz::sim::systems::MooringSystem">
///   <mooring_file>logs/mooring/lines.txt</mooring_file>
///   <recompute_unstretched_length>true</recompute_unstretched_length>
///   <unstretched_length_scale>1.15</unstretched_length_scale>
///   <slack_min>0.05</slack_min>
///   <slack_max>0.45</slack_max>
///   <break_tension>500000</break_tension>
///   <on_failure>stop</on_failure>
///   <water_depth>30</water_depth>
///   <tension_topic>/mooring/tension</tension_topic>
///   <marker_rate>30</marker_rate>
///   <waves enabled="true">
///     <component>
///       <amplitude>0.2</amplitude>
///       <omega>1.0</omega>
///       <wavenumber>0.1</wavenumber>
///       <phase>0.0</phase>
///       <dir_deg>0</dir_deg>
///     </component>
///   </waves>
///   <line>
///     <anchor>15 0 -30</anchor>
///     <unstretched_length>-1</unstretched_length>
///     <scale>1.15</scale>
///   </line>
///   <fairlead link="hull"><pos>0.55 0 -2.5</pos></fairlead>
/// </plugin>
/// \endcode
///
/// L'ordine di <line> e <fairlead> deve coincidere.
/// Valori strutturati (anchor, componente onda) come elementi figli; il
/// parser accetta anche la forma legacy ad attributi.
class Mooring
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemPostUpdate
{
  public: Mooring();
  public: ~Mooring() override;

  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) final;

  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override;

  public: void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) override;

  private: std::unique_ptr<MooringPrivate> dataPtr;
};

}  // namespace mooring
}  // namespace sim
}  // namespace gz

#endif  // GZ_MOORING_MOORING_HH_
