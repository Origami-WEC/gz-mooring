#include "gz/mooring/Mooring.hh"
#include "gz/mooring/WaveField.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport.hh>

#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/vector3d.pb.h>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

#include <sdf/Element.hh>

#include <MoorDyn2.h>

namespace gz
{
namespace sim
{
namespace mooring
{
namespace
{

constexpr int kMooringExitCode = 3;

std::string JoinPath(const std::string &_base, const std::string &_rel)
{
  if (_base.empty() || _rel.empty() || _rel[0] == '/')
    return _rel;
  if (_base.back() == '/')
    return _base + _rel;
  return _base + "/" + _rel;
}

bool FileExists(const std::string &_path)
{
  std::ifstream fh(_path);
  return fh.good();
}

std::string ResolveMooringFile(const std::string &_configured)
{
  if (_configured.empty())
    return _configured;
  if (_configured[0] == '/' && FileExists(_configured))
    return _configured;

  std::vector<std::string> candidates{_configured};
  if (const char *ws = std::getenv("MARITIME_WS"))
    candidates.push_back(JoinPath(ws, _configured));
  if (const char *ws = std::getenv("GZ_SIM_RESOURCE_PATH"))
  {
    std::string paths(ws);
    size_t start = 0;
    while (start < paths.size())
    {
      size_t end = paths.find(':', start);
      if (end == std::string::npos)
        end = paths.size();
      candidates.push_back(JoinPath(paths.substr(start, end - start),
                                   _configured));
      start = end + 1;
    }
  }
  for (const auto &c : candidates)
  {
    if (FileExists(c))
      return c;
  }
  return _configured;
}

bool IsFinite(double _v)
{
  return std::isfinite(_v);
}

bool IsFinite(const math::Vector3d &_v)
{
  return IsFinite(_v.X()) && IsFinite(_v.Y()) && IsFinite(_v.Z());
}

}  // namespace

struct Fairlead
{
  std::string linkName;
  Entity linkEntity{kNullEntity};
  math::Vector3d posLocal;
};

struct LineSpec
{
  math::Vector3d anchor;
  double unstretchedLength{-1.0};
  double scale{1.15};
};

class MooringPrivate
{
  public: Model model{kNullEntity};
  public: std::vector<Fairlead> fairleads;
  public: std::vector<LineSpec> lines;
  public: WaveField waveField;

  public: MoorDyn moordyn{nullptr};
  public: unsigned int nCoupledPoints{0};
  public: std::vector<double> x, xd, f;
  public: double moordynT{0.0};
  public: double moordynDt{0.0005};

  public: std::string mooringFile{"logs/mooring/lines.txt"};
  public: std::string resolvedMooringFile;
  public: std::string tensionTopic{"/mooring/tension"};
  public: std::string onFailure{"stop"};
  public: double markerRate{30.0};
  public: double waterDepth{0.0};
  public: double slackMin{0.05};
  public: double slackMax{0.45};
  public: double breakTension{0.0};
  public: bool recomputeUnstretched{true};
  public: int numSegsConfig{0};

  public: std::chrono::steady_clock::duration lastMarkerTime{
    std::chrono::steady_clock::duration::zero()};

  public: transport::Node node;
  public: transport::Node::Publisher tensionPub;
  public: std::vector<msgs::Marker> lineMarkers;
  public: std::vector<bool> lineBroken;

  public: bool mooringInitialized{false};
  public: bool wavesActive{false};
  public: bool initFailed{false};
  public: int preUpdateCount{0};
  public: std::vector<double> waveR, waveU, waveDU;

  public: bool Configure(const Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      EntityComponentManager &_ecm);
  public: void PreUpdate(const UpdateInfo &_info,
      EntityComponentManager &_ecm);
  public: void PostUpdate(const UpdateInfo &_info,
      const EntityComponentManager &_ecm);
  public: bool InitMooring(const UpdateInfo &_info,
      EntityComponentManager &_ecm);
  public: void UpdateMarkers(const UpdateInfo &_info,
      const EntityComponentManager &_ecm);

  public: bool ParseLineSpecs(const std::shared_ptr<const sdf::Element> &_sdf);
  public: bool ParseWaves(const std::shared_ptr<const sdf::Element> &_sdf);
  public: bool PatchUnstretchedLengths(const std::vector<double> &_lengths,
      const std::vector<int> &_numSegs,
      const std::vector<math::Vector3d> &_fairleadWorld);
  public: void ApplyWaveKinematics(double _t);
  public: void CheckLineBreaks(double _t);
  public: void Fatal(const std::string &_reason, double _t);

  public: math::Vector3d FairleadWorld(size_t _i,
      const EntityComponentManager &_ecm) const;
};

//////////////////////////////////////////////////
void MooringPrivate::Fatal(const std::string &_reason, double _t)
{
  std::ostringstream report;
  report << "MOORING FAILURE\n"
         << "reason: " << _reason << "\n"
         << "sim_time_s: " << _t << "\n"
         << "mooring_file: " << this->resolvedMooringFile << "\n"
         << "water_depth_m: " << this->waterDepth << "\n"
         << "n_lines: " << this->lines.size() << "\n"
         << "n_broken: "
         << std::count(this->lineBroken.begin(), this->lineBroken.end(), true)
         << "\n";

  for (size_t i = 0; i < this->fairleads.size(); ++i)
  {
    const double fx = this->f.size() >= (i + 1) * 3 ? this->f[i * 3] : 0.0;
    const double fy = this->f.size() >= (i + 1) * 3 ? this->f[i * 3 + 1] : 0.0;
    const double fz = this->f.size() >= (i + 1) * 3 ? this->f[i * 3 + 2] : 0.0;
    report << "fairlead[" << i << "] " << this->fairleads[i].linkName
           << " x=(" << this->x[i * 3] << ", " << this->x[i * 3 + 1] << ", "
           << this->x[i * 3 + 2] << ")"
           << " f=(" << fx << ", " << fy << ", " << fz << ")\n";
    if (this->moordyn && i < this->lines.size())
    {
      double ten = 0.0;
      MoorDynLine line = MoorDyn_GetLine(
          this->moordyn, static_cast<unsigned int>(i + 1));
      if (line)
        MoorDyn_GetLineFairTen(line, &ten);
      report << "line[" << i << "] tension_n: " << ten
             << " anchor=(" << this->lines[i].anchor.X() << ", "
             << this->lines[i].anchor.Y() << ", "
             << this->lines[i].anchor.Z() << ")\n";
    }
  }

  if (this->moordyn)
  {
    unsigned int nLines = 0;
    MoorDyn_GetNumberLines(this->moordyn, &nLines);
    for (unsigned int li = 0; li < nLines; ++li)
    {
      MoorDynLine line = MoorDyn_GetLine(this->moordyn, li + 1);
      if (!line)
        continue;
      unsigned int nNodes = 0;
      MoorDyn_GetLineNumberNodes(line, &nNodes);
      report << "nodes_line_" << li + 1 << ":\n";
      for (unsigned int n = 0; n < nNodes; ++n)
      {
        double p[3] = {0.0, 0.0, 0.0};
        MoorDyn_GetLineNodePos(line, n, p);
        report << "  " << n << ": " << p[0] << " " << p[1] << " " << p[2]
               << "\n";
      }
    }
  }

  gzerr << "Mooring: " << _reason << "\n"
        << "Mooring: report completo in logs/mooring/FAILURE.txt\n"
        << report.str();
  std::fflush(nullptr);

  std::string failurePath = "logs/mooring/FAILURE.txt";
  if (!this->resolvedMooringFile.empty())
  {
    const size_t slash = this->resolvedMooringFile.find_last_of('/');
    if (slash != std::string::npos)
      failurePath = this->resolvedMooringFile.substr(0, slash) +
          "/FAILURE.txt";
  }
  if (std::ofstream fh(failurePath); fh)
    fh << report.str();

  std::_Exit(kMooringExitCode);
}

//////////////////////////////////////////////////
math::Vector3d MooringPrivate::FairleadWorld(size_t _i,
    const EntityComponentManager &_ecm) const
{
  sim::Link link(this->fairleads[_i].linkEntity);
  auto pose = link.WorldPose(_ecm);
  if (!pose)
    return math::Vector3d::Zero;
  return pose->Rot().RotateVector(this->fairleads[_i].posLocal) + pose->Pos();
}

//////////////////////////////////////////////////
bool MooringPrivate::ParseLineSpecs(
    const std::shared_ptr<const sdf::Element> &_sdf)
{
  this->lines.clear();
  for (auto lineElem = _sdf->FindElement("line");
       lineElem;
       lineElem = lineElem->GetNextElement("line"))
  {
    LineSpec spec;
    // sdformat: attributo o elemento figlio (forma canonica: figlio)
    auto anchor = lineElem->Get<math::Vector3d>(
        "anchor", math::Vector3d::Zero);
    if (!anchor.second)
    {
      gzerr << "Mooring: <line> senza <anchor>\n";
      return false;
    }
    spec.anchor = anchor.first;

    auto len = lineElem->Get<double>("unstretched_length", -1.0);
    if (len.second)
      spec.unstretchedLength = len.first;

    auto scale = lineElem->Get<double>("scale", spec.scale);
    if (scale.second)
      spec.scale = scale.first;

    this->lines.push_back(spec);
  }
  return !this->lines.empty();
}

//////////////////////////////////////////////////
bool MooringPrivate::ParseWaves(
    const std::shared_ptr<const sdf::Element> &_sdf)
{
  auto wavesElem = _sdf->FindElement("waves");
  if (!wavesElem)
    return true;

  bool enabled = true;
  if (wavesElem->HasAttribute("enabled"))
    enabled = wavesElem->Get<bool>("enabled", true).first;
  if (!enabled)
    return true;

  std::vector<WaveField::Component> components;
  for (auto compElem = wavesElem->FindElement("component");
       compElem;
       compElem = compElem->GetNextElement("component"))
  {
    WaveField::Component c;
    auto get = [&compElem](const char *_name) -> double
    {
      return compElem->Get<double>(_name, 0.0).first;
    };
    c.amplitude = get("amplitude");
    c.omega = get("omega");
    c.waveNumber = get("wavenumber");
    c.phase = get("phase");
    const double dirDeg = get("dir_deg");
    c.dirX = std::cos(dirDeg * M_PI / 180.0);
    c.dirY = std::sin(dirDeg * M_PI / 180.0);
    if (c.amplitude > 0.0 && c.omega > 0.0 && c.waveNumber > 0.0)
      components.push_back(c);
  }
  this->waveField.SetComponents(std::move(components));
  return true;
}

//////////////////////////////////////////////////
bool MooringPrivate::PatchUnstretchedLengths(const std::vector<double> &_lengths,
    const std::vector<int> &_numSegs,
    const std::vector<math::Vector3d> &_fairleadWorld)
{
  std::ifstream in(this->resolvedMooringFile);
  if (!in)
  {
    gzerr << "Mooring: impossibile rileggere '"
          << this->resolvedMooringFile << "'\n";
    return false;
  }
  std::vector<std::string> rows;
  std::string row;
  while (std::getline(in, row))
    rows.push_back(row);
  in.close();

  bool inLines = false;
  bool inPoints = false;
  size_t patched = 0;
  for (auto &line : rows)
  {
    if (line.find("LINES") != std::string::npos &&
        line.find("OUTPUTS") == std::string::npos)
    {
      inLines = true;
      inPoints = false;
      continue;
    }
    if (line.find("POINT PROPERTIES") != std::string::npos)
    {
      inPoints = true;
      inLines = false;
      continue;
    }
    if (line.find("OPTIONS") != std::string::npos)
    {
      inLines = false;
      inPoints = false;
    }
    if (inPoints)
    {
      std::istringstream iss(line);
      int id = 0;
      std::string type;
      if (!(iss >> id >> type))
        continue;
      if (type != "Coupled")
        continue;
      const int fairleadIdx = (id - 2) / 2;
      if (fairleadIdx < 0 ||
          static_cast<size_t>(fairleadIdx) >= _fairleadWorld.size())
        continue;
      const math::Vector3d &p = _fairleadWorld[static_cast<size_t>(
          fairleadIdx)];
      double mass = 0.0, volume = 0.0, cda = 0.0, ca = 0.0;
      iss >> mass >> volume >> cda >> ca;
      std::ostringstream rewritten;
      rewritten << id << "   " << type << "  "
                << std::fixed << std::setprecision(6)
                << p.X() << " " << p.Y() << " " << p.Z()
                << " " << mass << " " << volume << " " << cda << " " << ca;
      line = rewritten.str();
      continue;
    }
    if (!inLines)
      continue;

    std::istringstream iss(line);
    int id = 0;
    if (!(iss >> id))
      continue;
    if (id < 1 || static_cast<size_t>(id) > _lengths.size())
      continue;

    std::string lineType, attachA, attachB, unstr, numSegs, outputs;
    if (!(iss >> lineType >> attachA >> attachB >> unstr >> numSegs))
      continue;
    outputs.clear();
    std::string rest;
    while (iss >> rest)
    {
      if (!outputs.empty())
        outputs += " ";
      outputs += rest;
    }

    std::ostringstream rewritten;
    rewritten << id << "  " << lineType << "    " << attachA << "       "
              << attachB << "       " << std::fixed << std::setprecision(4)
              << _lengths[static_cast<size_t>(id - 1)] << "  "
              << _numSegs[static_cast<size_t>(id - 1)];
    if (!outputs.empty())
      rewritten << " " << outputs;
    line = rewritten.str();
    ++patched;
  }

  if (patched != _lengths.size())
  {
    gzerr << "Mooring: patchate " << patched << " righe LINES attese "
          << _lengths.size() << "\n";
    return false;
  }

  std::ofstream out(this->resolvedMooringFile, std::ios::trunc);
  if (!out)
    return false;
  for (const auto &r : rows)
    out << r << "\n";
  return true;
}

//////////////////////////////////////////////////
bool MooringPrivate::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm)
{
  this->model = Model(_entity);

  if (_sdf->HasElement("mooring_file"))
    this->mooringFile = _sdf->Get<std::string>("mooring_file");
  if (_sdf->HasElement("tension_topic"))
    this->tensionTopic = _sdf->Get<std::string>("tension_topic");
  if (_sdf->HasElement("marker_rate"))
    this->markerRate = _sdf->Get<double>("marker_rate");
  if (_sdf->HasElement("water_depth"))
    this->waterDepth = _sdf->Get<double>("water_depth");
  if (_sdf->HasElement("slack_min"))
    this->slackMin = _sdf->Get<double>("slack_min");
  if (_sdf->HasElement("slack_max"))
    this->slackMax = _sdf->Get<double>("slack_max");
  if (_sdf->HasElement("break_tension"))
    this->breakTension = _sdf->Get<double>("break_tension");
  if (_sdf->HasElement("on_failure"))
    this->onFailure = _sdf->Get<std::string>("on_failure");
  if (_sdf->HasElement("recompute_unstretched_length"))
  {
    this->recomputeUnstretched =
        _sdf->Get<bool>("recompute_unstretched_length");
  }
  if (_sdf->HasElement("num_segs"))
    this->numSegsConfig = _sdf->Get<int>("num_segs");

  if (!this->ParseLineSpecs(_sdf))
  {
    gzerr << "Mooring: servono elementi <line> con <anchor>\n";
    return false;
  }
  if (!this->ParseWaves(_sdf))
    return false;

  for (auto fairleadElem = _sdf->FindElement("fairlead");
       fairleadElem;
       fairleadElem = fairleadElem->GetNextElement("fairlead"))
  {
    Fairlead fl;
    fl.linkName = fairleadElem->Get<std::string>("link", "hull").first;
    auto posElem = fairleadElem->FindElement("pos");
    if (!posElem)
    {
      gzerr << "Mooring: <fairlead> senza <pos>\n";
      return false;
    }
    fl.posLocal = posElem->Get<math::Vector3d>();

    fl.linkEntity = this->model.LinkByName(_ecm, fl.linkName);
    if (fl.linkEntity == kNullEntity)
    {
      gzerr << "Mooring: link '" << fl.linkName << "' non trovato\n";
      return false;
    }
    sim::Link link(fl.linkEntity);
    link.EnableVelocityChecks(_ecm);
    this->fairleads.push_back(fl);
  }

  if (this->fairleads.empty())
  {
    gzerr << "Mooring: nessun fairlead configurato\n";
    return false;
  }
  if (this->fairleads.size() != this->lines.size())
  {
    gzerr << "Mooring: " << this->fairleads.size() << " fairlead ma "
          << this->lines.size() << " <line>\n";
    return false;
  }

  this->lineBroken.assign(this->lines.size(), false);
  this->tensionPub = this->node.Advertise<msgs::Float_V>(this->tensionTopic);
  this->resolvedMooringFile = ResolveMooringFile(this->mooringFile);

  gzmsg << "Mooring: " << this->fairleads.size() << " linee, file "
        << this->resolvedMooringFile << ", tension topic "
        << this->tensionTopic << ", on_failure=" << this->onFailure << "\n";
  return true;
}

//////////////////////////////////////////////////
bool MooringPrivate::InitMooring(const UpdateInfo &/*_info*/,
    EntityComponentManager &_ecm)
{
  std::vector<double> lengths(this->lines.size(), 0.0);
  std::vector<int> numSegs(this->lines.size(), 24);
  std::vector<math::Vector3d> fairleadPositions(this->lines.size());

  for (size_t i = 0; i < this->lines.size(); ++i)
  {
    const math::Vector3d fairleadW = this->FairleadWorld(i, _ecm);
    fairleadPositions[i] = fairleadW;
    const double dist = (this->lines[i].anchor - fairleadW).Length();
    if (dist < 1e-6)
    {
      gzerr << "Mooring: linea " << i + 1
            << " ha ancora e fairlead coincidenti\n";
      return false;
    }

    double length = 0.0;
    if (this->recomputeUnstretched || this->lines[i].unstretchedLength <= 0.0)
      length = this->lines[i].scale * dist;
    else
      length = this->lines[i].unstretchedLength;

    const double slack = (length - dist) / length;
    if (slack < this->slackMin || slack > this->slackMax)
    {
      std::ostringstream oss;
      oss << "linea " << i + 1 << " slack " << 100.0 * slack
          << "% fuori da [" << 100.0 * this->slackMin << ", "
          << 100.0 * this->slackMax << "] (UnstrLen=" << length
          << " m, dist=" << dist << " m). "
          << "Aggiorna anchor_world/unstretched_length in config/mooring.json";
      if (this->onFailure == "disable")
      {
        gzerr << "Mooring: " << oss.str() << "\n";
        return false;
      }
      this->Fatal(oss.str(),
          std::chrono::duration<double>(
              std::chrono::steady_clock::duration::zero()).count());
    }
    lengths[i] = length;
    if (this->numSegsConfig > 0)
      numSegs[i] = this->numSegsConfig;
    else
      numSegs[i] = std::min(40, std::max(16,
          static_cast<int>(std::ceil(length / 1.5))));
  }

  if (!FileExists(this->resolvedMooringFile))
  {
    std::ostringstream oss;
    oss << "file lines.txt non trovato. Tentati: '" << this->mooringFile
        << "', $MARITIME_WS/" << this->mooringFile
        << ", $GZ_SIM_RESOURCE_PATH/... — cwd corrente li ignora";
    if (this->onFailure == "disable")
    {
      gzerr << "Mooring: " << oss.str() << "\n";
      this->initFailed = true;
      return false;
    }
    this->Fatal(oss.str(), 0.0);
  }
  if (this->recomputeUnstretched &&
      !this->PatchUnstretchedLengths(lengths, numSegs, fairleadPositions))
  {
    if (this->onFailure == "disable")
    {
      this->initFailed = true;
      return false;
    }
    this->Fatal("impossibile aggiornare UnstrLen in lines.txt", 0.0);
  }

  this->moordyn = MoorDyn_Create(this->resolvedMooringFile.c_str());
  if (!this->moordyn)
  {
    gzerr << "Mooring: MoorDyn_Create fallito per '"
          << this->resolvedMooringFile << "'\n";
    return false;
  }
  MoorDyn_SetVerbosity(this->moordyn, MOORDYN_WRN_LEVEL);

  unsigned int ndof = 0;
  if (MoorDyn_NCoupledDOF(this->moordyn, &ndof) != MOORDYN_SUCCESS)
  {
    gzerr << "Mooring: MoorDyn_NCoupledDOF fallito\n";
    MoorDyn_Close(this->moordyn);
    this->moordyn = nullptr;
    return false;
  }

  this->nCoupledPoints = ndof / 3;
  if (this->nCoupledPoints != this->fairleads.size())
  {
    gzerr << "Mooring: " << this->nCoupledPoints
          << " Coupled in lines.txt ma " << this->fairleads.size()
          << " fairlead in SDF\n";
    MoorDyn_Close(this->moordyn);
    this->moordyn = nullptr;
    return false;
  }

  this->x.assign(ndof, 0.0);
  this->xd.assign(ndof, 0.0);
  this->f.assign(ndof, 0.0);

  for (size_t i = 0; i < this->fairleads.size(); ++i)
  {
    const math::Vector3d pW = this->FairleadWorld(i, _ecm);
    if (!IsFinite(pW))
    {
      gzerr << "Mooring: pose non finita per '"
            << this->fairleads[i].linkName << "'\n";
      MoorDyn_Close(this->moordyn);
      this->moordyn = nullptr;
      return false;
    }
    this->x[i * 3 + 0] = pW.X();
    this->x[i * 3 + 1] = pW.Y();
    this->x[i * 3 + 2] = pW.Z();
  }

  if (MoorDyn_Init(this->moordyn, this->x.data(), this->xd.data())
      != MOORDYN_SUCCESS)
  {
    gzerr << "Mooring: MoorDyn_Init fallito\n";
    MoorDyn_Close(this->moordyn);
    this->moordyn = nullptr;
    return false;
  }

  this->wavesActive = !this->waveField.Empty();
  if (this->wavesActive)
  {
    unsigned int nwp = 0;
    if (MoorDyn_ExternalWaveKinInit(this->moordyn, &nwp) != MOORDYN_SUCCESS)
    {
      gzerr << "Mooring: MoorDyn_ExternalWaveKinInit fallito\n";
      MoorDyn_Close(this->moordyn);
      this->moordyn = nullptr;
      return false;
    }
    this->waveR.assign(3 * nwp, 0.0);
    this->waveU.assign(3 * nwp, 0.0);
    this->waveDU.assign(3 * nwp, 0.0);
  }

  this->mooringInitialized = true;
  gzmsg << "Mooring: MoorDyn inizializzato (" << this->nCoupledPoints
        << " Coupled, " << ndof << " DOF"
        << (this->wavesActive ? ", onde esterne attive" : ", acqua calma")
        << ")\n";
  for (size_t i = 0; i < lengths.size(); ++i)
  {
    const double dist =
        (this->lines[i].anchor - math::Vector3d(this->x[i * 3],
            this->x[i * 3 + 1], this->x[i * 3 + 2])).Length();
    gzmsg << "Mooring: linea " << i + 1 << " UnstrLen=" << lengths[i]
          << " m, dist=" << dist << " m, slack="
          << 100.0 * (lengths[i] - dist) / lengths[i] << "%\n";
  }
  return true;
}

//////////////////////////////////////////////////
void MooringPrivate::ApplyWaveKinematics(double _t)
{
  if (!this->wavesActive || !this->moordyn)
    return;

  // BreakLine aggiunge punti liberi a MoorDyn: il conteggio npW preso a init
  // diventa stale e MoorDyn_ExternalWaveKinSet/SetWaveKinematics esplode con
  // "not enough points" (e GetCoordinates scrivebbe fuori buffer). Riallinea
  // npW e i buffer a ogni chiamata: ExternalWaveKinInit e' solo un recount.
  unsigned int nwp = 0;
  if (MoorDyn_ExternalWaveKinInit(this->moordyn, &nwp) != MOORDYN_SUCCESS)
    return;
  if (!nwp)
    return;
  if (this->waveR.size() != 3 * nwp)
  {
    this->waveR.assign(3 * nwp, 0.0);
    this->waveU.assign(3 * nwp, 0.0);
    this->waveDU.assign(3 * nwp, 0.0);
  }

  if (MoorDyn_ExternalWaveKinGetCoordinates(this->moordyn,
        this->waveR.data()) != MOORDYN_SUCCESS)
    return;

  const size_t n = nwp;
  for (size_t i = 0; i < n; ++i)
  {
    const math::Vector3d p(this->waveR[i * 3], this->waveR[i * 3 + 1],
                           this->waveR[i * 3 + 2]);
    math::Vector3d u, du;
    this->waveField.Evaluate(_t, p, this->waterDepth, u, du);
    this->waveU[i * 3 + 0] = u.X();
    this->waveU[i * 3 + 1] = u.Y();
    this->waveU[i * 3 + 2] = u.Z();
    this->waveDU[i * 3 + 0] = du.X();
    this->waveDU[i * 3 + 1] = du.Y();
    this->waveDU[i * 3 + 2] = du.Z();
  }
  MoorDyn_ExternalWaveKinSet(this->moordyn, this->waveU.data(),
                             this->waveDU.data(), _t);
}

//////////////////////////////////////////////////
void MooringPrivate::CheckLineBreaks(double _t)
{
  if (this->breakTension <= 0.0 || !this->moordyn)
    return;

  for (size_t i = 0; i < this->lines.size(); ++i)
  {
    if (this->lineBroken[i])
      continue;
    MoorDynLine line = MoorDyn_GetLine(this->moordyn,
                                       static_cast<unsigned int>(i + 1));
    if (!line)
      continue;
    double ten = 0.0;
    MoorDyn_GetLineFairTen(line, &ten);
    if (ten <= this->breakTension)
      continue;

    const unsigned int pointId = static_cast<unsigned int>(i * 2 + 2);
    MoorDynPoint point = MoorDyn_GetPoint(this->moordyn, pointId);
    if (point && MoorDyn_BreakLine(this->moordyn, point, line)
        == MOORDYN_SUCCESS)
    {
      this->lineBroken[i] = true;
      gzwarn << "Mooring: linea " << i + 1 << " ROTTA a t=" << _t
             << " s (tensione " << ten << " N > " << this->breakTension
             << " N)\n";
    }
  }
}

//////////////////////////////////////////////////
void MooringPrivate::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (this->initFailed)
    return;
  // Salta il primo PreUpdate: altri plugin (es. WecDynamics) possono
  // applicare JointPositionReset al primo passo. La pose letta prima del
  // physics step non e' quella reale: inizializzare MoorDyn con quella
  // posizione causa uno snap dei fairlead al passo successivo e un'esplosione
  // numerica. Al secondo PreUpdate la fisica ha gia' applicato i reset.
  ++this->preUpdateCount;
  if (this->preUpdateCount < 2)
    return;
  if (!this->mooringInitialized)
  {
    if (!this->InitMooring(_info, _ecm))
      return;
  }
  if (!this->moordyn)
    return;

  for (size_t i = 0; i < this->fairleads.size(); ++i)
  {
    sim::Link link(this->fairleads[i].linkEntity);
    auto pose = link.WorldPose(_ecm);
    if (!pose)
      continue;

    const math::Vector3d pW =
        pose->Rot().RotateVector(this->fairleads[i].posLocal) + pose->Pos();
    auto vW = link.WorldLinearVelocity(_ecm, this->fairleads[i].posLocal);
    const math::Vector3d v = vW.value_or(math::Vector3d::Zero);

    this->x[i * 3 + 0] = pW.X();
    this->x[i * 3 + 1] = pW.Y();
    this->x[i * 3 + 2] = pW.Z();
    this->xd[i * 3 + 0] = v.X();
    this->xd[i * 3 + 1] = v.Y();
    this->xd[i * 3 + 2] = v.Z();
  }

  this->moordynT = std::chrono::duration<double>(_info.simTime).count();
  this->moordynDt = std::chrono::duration<double>(_info.dt).count();
  if (this->moordynDt <= 0.0)
    return;

  this->ApplyWaveKinematics(this->moordynT);

  if (MoorDyn_Step(this->moordyn, this->x.data(), this->xd.data(),
                   this->f.data(), &this->moordynT, &this->moordynDt)
      != MOORDYN_SUCCESS)
  {
    std::ostringstream oss;
    oss << "MoorDyn_Step fallito a t=" << this->moordynT << " s";
    if (this->onFailure == "disable")
    {
      gzwarn << "Mooring: " << oss.str() << ", forze azzerate\n";
      std::fill(this->f.begin(), this->f.end(), 0.0);
      return;
    }
    this->Fatal(oss.str(), this->moordynT);
  }

  for (double v : this->f)
  {
    if (!IsFinite(v))
    {
      std::ostringstream oss;
      oss << "forze di ormeggio non finite a t=" << this->moordynT << " s";
      if (this->onFailure == "disable")
      {
        gzwarn << "Mooring: " << oss.str() << ", forze azzerate\n";
        std::fill(this->f.begin(), this->f.end(), 0.0);
        return;
      }
      this->Fatal(oss.str(), this->moordynT);
    }
  }

  this->CheckLineBreaks(this->moordynT);

  for (size_t i = 0; i < this->fairleads.size(); ++i)
  {
    if (i < this->lineBroken.size() && this->lineBroken[i])
      continue;

    const math::Vector3d fi(this->f[i * 3], this->f[i * 3 + 1],
                            this->f[i * 3 + 2]);
    if (fi == math::Vector3d::Zero)
      continue;
    // Applica la forza nel fairlead usando la overload con offset:
    // AddWorldWrench(force, torque) applica attorno al link origin, ma il
    // motore fisico risolve attorno al COM. Con offset = posLocal la forza
    // e' applicata nel punto giusto senza calcolare il momento a mano.
    sim::Link applyLink(this->fairleads[i].linkEntity);
    applyLink.AddWorldWrench(_ecm, fi, math::Vector3d::Zero,
                             this->fairleads[i].posLocal);
  }
}

//////////////////////////////////////////////////
void MooringPrivate::UpdateMarkers(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  double now = std::chrono::duration<double>(_info.simTime).count();
  double period = 1.0 / std::max(this->markerRate, 1.0);
  if (this->lastMarkerTime != std::chrono::steady_clock::duration::zero())
  {
    double last = std::chrono::duration<double>(this->lastMarkerTime).count();
    if ((now - last) < period)
      return;
  }
  this->lastMarkerTime = _info.simTime;

  msgs::Float_V tensionMsg;
  unsigned int nLines = 0;
  MoorDyn_GetNumberLines(this->moordyn, &nLines);
  for (size_t i = 0; i < this->fairleads.size(); ++i)
  {
    double t = 0.0;
    if (i < nLines && !(i < this->lineBroken.size() && this->lineBroken[i]))
    {
      MoorDynLine line = MoorDyn_GetLine(this->moordyn,
        static_cast<unsigned int>(i + 1));
      if (line)
        MoorDyn_GetLineFairTen(line, &t);
    }
    tensionMsg.add_data(t);
  }
  this->tensionPub.Publish(tensionMsg);

  if (this->lineMarkers.size() != nLines)
    this->lineMarkers.resize(nLines);

  for (unsigned int li = 0; li < nLines; ++li)
  {
    auto &msg = this->lineMarkers[li];
    msg.set_ns("mooring");
    msg.set_id(static_cast<int>(li + 1));
    msg.mutable_point()->Clear();

    if (li < this->lineBroken.size() && this->lineBroken[li])
    {
      msg.set_action(msgs::Marker::DELETE_MARKER);
      this->node.Request("/marker", msg);
      continue;
    }

    MoorDynLine line = MoorDyn_GetLine(this->moordyn, li + 1);
    if (!line)
      continue;

    msg.set_action(msgs::Marker::ADD_MODIFY);
    msg.set_type(msgs::Marker::LINE_LIST);
    msg.set_visibility(msgs::Marker::GUI);

    msgs::Set(msg.mutable_material()->mutable_ambient(),
              math::Color(0.9, 0.7, 0.1, 1.0));
    msgs::Set(msg.mutable_material()->mutable_diffuse(),
              math::Color(0.9, 0.7, 0.1, 1.0));

    unsigned int nNodes = 0;
    MoorDyn_GetLineNumberNodes(line, &nNodes);
    double p0[3], p1[3];
    for (unsigned int n = 0; n + 1 < nNodes; ++n)
    {
      MoorDyn_GetLineNodePos(line, n, p0);
      MoorDyn_GetLineNodePos(line, n + 1, p1);
      msgs::Set(msg.add_point(), math::Vector3d(p0[0], p0[1], p0[2]));
      msgs::Set(msg.add_point(), math::Vector3d(p1[0], p1[1], p1[2]));
    }
    this->node.Request("/marker", msg);
  }
}

//////////////////////////////////////////////////
void MooringPrivate::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  if (!this->moordyn)
    return;
  this->UpdateMarkers(_info, _ecm);
}

//////////////////////////////////////////////////
Mooring::Mooring() : dataPtr(std::make_unique<MooringPrivate>())
{
}

//////////////////////////////////////////////////
Mooring::~Mooring()
{
  if (this->dataPtr->moordyn)
    MoorDyn_Close(this->dataPtr->moordyn);
}

//////////////////////////////////////////////////
void Mooring::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  if (!this->dataPtr->Configure(_entity, _sdf, _ecm))
  {
    gzerr << "Mooring: configurazione fallita, ormeggio inattivo\n";
    this->dataPtr->initFailed = true;
  }
}

//////////////////////////////////////////////////
void Mooring::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  this->dataPtr->PreUpdate(_info, _ecm);
}

//////////////////////////////////////////////////
void Mooring::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  this->dataPtr->PostUpdate(_info, _ecm);
}

}  // namespace mooring
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(gz::sim::mooring::Mooring,
              gz::sim::System,
              gz::sim::mooring::Mooring::ISystemConfigure,
              gz::sim::mooring::Mooring::ISystemPreUpdate,
              gz::sim::mooring::Mooring::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::mooring::Mooring,
                    "gz::sim::systems::MooringSystem")
