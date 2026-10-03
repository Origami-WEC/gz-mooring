#ifndef GZ_MOORING_WAVEFIELD_HH_
#define GZ_MOORING_WAVEFIELD_HH_

#include <cmath>
#include <vector>

#include <gz/math/Vector3.hh>

namespace gz
{
namespace sim
{
namespace mooring
{

/// \brief Somma di componenti di Airy (onde lineari ad ampiezza finita).
///
/// Ogni componente è caratterizzata da ampiezza, pulsazione, numero d'onda,
/// fase e direzione di propagazione. La cinematica è valutata nei nodi dei
/// cavi e fornita a MoorDyn tramite MoorDyn_ExternalWaveKinSet.
class WaveField
{
  public: struct Component
  {
    double amplitude{0.0};
    double omega{0.0};
    double waveNumber{0.0};
    double phase{0.0};
    double dirX{1.0};
    double dirY{0.0};
  };

  public: void SetComponents(std::vector<Component> _components)
  {
    this->components = std::move(_components);
  }

  public: bool Empty() const
  {
    return this->components.empty();
  }

  public: const std::vector<Component> &Components() const
  {
    return this->components;
  }

  public: void Evaluate(double _t,
                        const math::Vector3d &_p,
                        double _waterDepth,
                        math::Vector3d &_u,
                        math::Vector3d &_du) const
  {
    _u = math::Vector3d::Zero;
    _du = math::Vector3d::Zero;
    if (this->components.empty() || _waterDepth <= 0.0)
      return;

    for (const auto &c : this->components)
    {
      const double k = c.waveNumber;
      const double kd = k * _waterDepth;
      const double sinhKd = std::sinh(std::max(kd, 1e-9));
      const double zRel = _p.Z() + _waterDepth;
      const double kz = std::min(std::max(k * zRel, -kd), kd);
      const double phase = k * (c.dirX * _p.X() + c.dirY * _p.Y())
          - c.omega * _t + c.phase;
      const double cosP = std::cos(phase);
      const double sinP = std::sin(phase);

      const double decay = std::cosh(kz) / sinhKd;
      const double vertical = std::sinh(kz) / sinhKd;
      const double uAmp = c.amplitude * c.omega * decay;
      const double wAmp = c.amplitude * c.omega * vertical;

      _u.X() += uAmp * cosP * c.dirX;
      _u.Y() += uAmp * cosP * c.dirY;
      _u.Z() += wAmp * sinP;

      _du.X() += c.omega * uAmp * sinP * c.dirX;
      _du.Y() += c.omega * uAmp * sinP * c.dirY;
      _du.Z() += -c.omega * wAmp * cosP;
    }
  }

  private: std::vector<Component> components;
};

}  // namespace mooring
}  // namespace sim
}  // namespace gz

#endif  // GZ_MOORING_WAVEFIELD_HH_
