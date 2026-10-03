// Copyright (C) 2026  maritime_ws
//
// WecDashboard -- implementazione. Vedi WecDashboard.hh.

#include "WecDashboard.hh"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/msgs/float_v.pb.h>
#include <gz/msgs/model.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/world_stats.pb.h>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QQuickItem>
#include <QQuickWindow>
#include <QRegularExpression>
#include <QString>
#include <QTimer>

namespace wec
{

namespace
{

QString ReadFile(const QString &_path)
{
  QFile fh(_path);
  if (!fh.open(QIODevice::ReadOnly | QIODevice::Text))
    return {};
  return QString::fromUtf8(fh.readAll());
}

/// Risolve un path relativo a MARITIME_WS (stessa convenzione di Mooring).
QString ResolvePath(const QString &_rel)
{
  if (_rel.isEmpty())
    return _rel;
  if (_rel.startsWith('/'))
    return _rel;
  const QByteArray ws = qgetenv("MARITIME_WS");
  if (!ws.isEmpty())
    return QString::fromUtf8(ws) + "/" + _rel;
  return _rel;
}

/// Rimuove le chiavi `$comment*` da un oggetto JSON (ricorsivo).
QJsonValue StripComments(const QJsonValue &_v)
{
  if (_v.isObject())
  {
    const QJsonObject obj = _v.toObject();
    QJsonObject out;
    for (auto it = obj.constBegin(); it != obj.constEnd(); ++it)
    {
      if (it.key().startsWith("$comment"))
        continue;
      out.insert(it.key(), StripComments(it.value()));
    }
    return out;
  }
  if (_v.isArray())
  {
    const QJsonArray arr = _v.toArray();
    QJsonArray out;
    for (const auto &e : arr)
      out.append(StripComments(e));
    return out;
  }
  return _v;
}

QJsonObject LoadJsonFile(const QString &_path)
{
  const QString text = ReadFile(_path);
  if (text.isEmpty())
  {
    gzerr << "WecDashboard: impossibile leggere " << _path.toStdString() << "\n";
    return {};
  }
  QJsonParseError err{};
  const QJsonDocument doc = QJsonDocument::fromJson(text.toUtf8(), &err);
  if (err.error != QJsonParseError::NoError)
  {
    gzerr << "WecDashboard: JSON non valido in " << _path.toStdString()
          << ": " << err.errorString().toStdString() << "\n";
    return {};
  }
  return StripComments(doc.object()).toObject();
}

template <typename T>
bool RequireNumber(const QJsonObject &_obj, const QString &_key, T &_out)
{
  if (!_obj.contains(_key) || !_obj.value(_key).isDouble())
  {
    gzerr << "WecDashboard: chiave di configurazione assente o non numerica: '"
          << _key.toStdString() << "'\n";
    return false;
  }
  _out = static_cast<T>(_obj.value(_key).toDouble());
  return true;
}

bool RequireString(const QJsonObject &_obj, const QString &_key, QString &_out)
{
  if (!_obj.contains(_key) || !_obj.value(_key).isString())
  {
    gzerr << "WecDashboard: chiave di configurazione assente o non stringa: '"
          << _key.toStdString() << "'\n";
    return false;
  }
  _out = _obj.value(_key).toString();
  return true;
}

/// Piccolo buffer circolare per le serie storiche.
struct Series
{
  std::vector<double> t;
  std::vector<double> v;

  void Push(double _time, double _value, double _span)
  {
    t.push_back(_time);
    v.push_back(_value);
    while (!t.empty() && (t.back() - t.front()) > _span)
    {
      t.erase(t.begin());
      v.erase(v.begin());
    }
  }

  void Clear()
  {
    t.clear();
    v.clear();
  }
};

/// PSD monolatera via finestra di Hann (coerente con telemetry.analyze.spectrum).
void ComputePsd(const std::vector<double> &_t, const std::vector<double> &_x,
                double _fMin, double _fMax,
                std::vector<double> &_freqs, std::vector<double> &_psd)
{
  _freqs.clear();
  _psd.clear();
  const size_t n = _t.size();
  if (n < 16)
    return;

  // resample su griglia regolare
  const double span = _t.back() - _t.front();
  if (span <= 0.0)
    return;
  size_t nfft = 256;
  while (nfft < n)
    nfft <<= 1;

  std::vector<double> x(nfft, 0.0);
  const double dt = span / static_cast<double>(nfft - 1);
  for (size_t i = 0; i < nfft; ++i)
  {
    const double ti = _t.front() + dt * static_cast<double>(i);
    size_t j = 0;
    while (j + 1 < n && _t[j + 1] < ti)
      ++j;
    if (j + 1 < n)
    {
      const double denom = _t[j + 1] - _t[j];
      const double a = (denom > 0.0) ? (ti - _t[j]) / denom : 0.0;
      x[i] = _x[j] * (1.0 - a) + _x[j + 1] * a;
    }
    else
      x[i] = _x.back();
  }
  double mean = 0.0;
  for (double v : x)
    mean += v;
  mean /= static_cast<double>(nfft);
  double winSum = 0.0;
  for (size_t i = 0; i < nfft; ++i)
  {
    const double w = 0.5 - 0.5 * std::cos(
        2.0 * M_PI * static_cast<double>(i) / static_cast<double>(nfft - 1));
    x[i] = (x[i] - mean) * w;
    winSum += w * w;
  }

  // DFT reale (decimata ai bin in banda per il plotting)
  const double fs = 1.0 / dt;
  const double df = fs / static_cast<double>(nfft);
  for (size_t k = 1; k < nfft / 2; ++k)
  {
    const double f = df * static_cast<double>(k);
    if (f < _fMin || f > _fMax)
      continue;
    double re = 0.0, im = 0.0;
    for (size_t i = 0; i < nfft; ++i)
    {
      const double ang = -2.0 * M_PI * static_cast<double>(k * i) /
                         static_cast<double>(nfft);
      re += x[i] * std::cos(ang);
      im += x[i] * std::sin(ang);
    }
    double p = (re * re + im * im) / (fs * winSum);
    _freqs.push_back(f);
    _psd.push_back(p);
  }
}

}  // namespace

class WecDashboardPrivate
{
  public: bool LoadConfiguration();
  public: void Subscribe();
  public: void DiscoverWorld();
  public: void OnWorlds(const gz::msgs::StringMsg_V &_reply, const bool _result);
  public: void OnOdometry(const gz::msgs::Odometry &_msg);
  public: void OnJointState(const gz::msgs::Model &_msg);
  public: void OnPtoState(const gz::msgs::Float_V &_msg);
  public: void OnAccel(const gz::msgs::Vector3d &_msg);
  public: void OnAngAccel(const gz::msgs::Vector3d &_msg);
  public: void OnTension(const gz::msgs::Float_V &_msg);
  public: void OnStats(const gz::msgs::WorldStatistics &_msg);
  public: void PushSample(double _time);
  public: void RefreshPsd();
  public: void RefreshCaches();

  public: std::mutex mutex;

  // config
  public: QString dashboardPath;
  public: QString pipelinePath;
  public: QString hydroPath;
  public: QString wavesPath;
  public: QString equilibriumPath;
  public: QJsonObject dashboard;
  public: QJsonObject pipeline;
  public: QJsonObject equilibrium;
  public: double fMinHz{0.0};
  public: double fMaxHz{0.0};
  public: double historyS{0.0};
  public: double windowS{0.0};
  public: double decimateHz{0.0};
  public: double psdWindowS{0.0};
  public: double ptoDamping{0.0};
  public: double heaveEqZ{0.0};
  public: double uiRefreshHz{20.0};
  public: QString odomTopic;
  public: QString jointTopic;
  public: QString tensionTopic;
  public: QString accelTopic;
  public: QString angAccelTopic;
  public: QString ptoStateTopic;

  // runtime
  public: bool configured{false};
  public: bool paused{false};
  public: std::atomic<bool> worldSubscribed{false};
  public: double lastSampleTime{-1.0};
  public: double lastPsdTime{-1.0};
  public: double simTime{0.0};
  public: std::map<std::string, Series> series;
  public: std::map<std::string, double> values;
  public: std::map<std::string, std::vector<double>> psd;
  public: std::vector<double> psdFreqs;
  public: QString seriesJsonCache;
  public: QString valuesJsonCache;
  public: QString psdJsonCache;

  public: gz::transport::Node node;
};

//////////////////////////////////////////////////
bool WecDashboardPrivate::LoadConfiguration()
{
  // i percorsi dei config sono dichiarati nel config stesso (paths in
  // pipeline.json) -- nessun path hard-coded.
  const QByteArray ws = qgetenv("MARITIME_WS");
  QString pipelineRel = "config/pipeline.json";
  if (!ws.isEmpty())
  {
    const QString candidate = QString::fromUtf8(ws) + "/" + pipelineRel;
    if (QFile::exists(candidate))
      pipelineRel = "config/pipeline.json";
  }
  this->pipelinePath = ResolvePath(pipelineRel);
  this->pipeline = LoadJsonFile(this->pipelinePath);
  if (this->pipeline.isEmpty())
    return false;

  const QJsonObject paths = this->pipeline.value("paths").toObject();
  QString dashRel, hydroRel, wavesRel, eqRel;
  if (!RequireString(paths, "dashboard_config", dashRel) ||
      !RequireString(paths, "hydro_config", hydroRel) ||
      !RequireString(paths, "waves_config", wavesRel) ||
      !RequireString(paths, "equilibrium_json", eqRel))
    return false;
  this->dashboardPath = ResolvePath(dashRel);
  this->hydroPath = ResolvePath(hydroRel);
  this->wavesPath = ResolvePath(wavesRel);
  this->equilibriumPath = ResolvePath(eqRel);

  this->dashboard = LoadJsonFile(this->dashboardPath);
  if (this->dashboard.isEmpty())
    return false;
  this->equilibrium = LoadJsonFile(this->equilibriumPath);

  // banda: unica sorgente di frequenze (derivata dai periodi)
  const QJsonObject band = this->pipeline.value("wave_band").toObject();
  double tMin = 0.0, tMax = 0.0;
  if (!RequireNumber(band, "period_min_s", tMin) ||
      !RequireNumber(band, "period_max_s", tMax))
    return false;
  if (tMin <= 0.0 || tMax <= tMin)
  {
    gzerr << "WecDashboard: wave_band.period_* non validi\n";
    return false;
  }
  this->fMinHz = 1.0 / tMax;
  this->fMaxHz = 1.0 / tMin;

  if (!RequireNumber(this->dashboard, "history_s", this->historyS) ||
      !RequireNumber(this->dashboard, "window_s", this->windowS) ||
      !RequireNumber(this->dashboard, "render_decimate_hz", this->decimateHz) ||
      !RequireNumber(this->dashboard, "psd_window_s", this->psdWindowS))
    return false;

  if (this->dashboard.contains("ui_refresh_hz") &&
      this->dashboard.value("ui_refresh_hz").isDouble() &&
      this->dashboard.value("ui_refresh_hz").toDouble() > 0.0)
    this->uiRefreshHz = this->dashboard.value("ui_refresh_hz").toDouble();

  const QJsonObject tel = this->pipeline.value("telemetry").toObject();
  if (!RequireNumber(tel, "pto_damping_Ns_m", this->ptoDamping))
    return false;
  if (!RequireString(tel, "odom_topic", this->odomTopic) ||
      !RequireString(tel, "joint_state_topic", this->jointTopic) ||
      !RequireString(tel, "mooring_tension_topic", this->tensionTopic) ||
      !RequireString(tel, "accel_topic", this->accelTopic) ||
      !RequireString(tel, "angular_accel_topic", this->angAccelTopic) ||
      !RequireString(tel, "pto_state_topic", this->ptoStateTopic))
    return false;

  // topic joint_state contiene {world}: si scopre dal nome world a runtime
  this->heaveEqZ =
      this->equilibrium.contains("waterline_z_m")
          ? -this->equilibrium.value("waterline_z_m").toDouble()
          : 0.0;

  return true;
}

//////////////////////////////////////////////////
void WecDashboardPrivate::Subscribe()
{
  this->node.Subscribe(this->odomTopic.toStdString(),
                       &WecDashboardPrivate::OnOdometry, this);
  this->node.Subscribe(this->tensionTopic.toStdString(),
                       &WecDashboardPrivate::OnTension, this);
  this->node.Subscribe(this->accelTopic.toStdString(),
                       &WecDashboardPrivate::OnAccel, this);
  this->node.Subscribe(this->angAccelTopic.toStdString(),
                       &WecDashboardPrivate::OnAngAccel, this);
  this->node.Subscribe(this->ptoStateTopic.toStdString(),
                       &WecDashboardPrivate::OnPtoState, this);
}

//////////////////////////////////////////////////
void WecDashboardPrivate::DiscoverWorld()
{
  if (this->worldSubscribed)
    return;
  this->node.Request("/gazebo/worlds", &WecDashboardPrivate::OnWorlds, this);
}

//////////////////////////////////////////////////
void WecDashboardPrivate::OnWorlds(const gz::msgs::StringMsg_V &_reply,
                                   const bool _result)
{
  if (!_result || _reply.data_size() == 0)
    return;
  if (this->worldSubscribed.exchange(true))
    return;

  const std::string worldName = _reply.data(0);
  std::string topic = this->jointTopic.toStdString();
  const std::string token = "{world}";
  const auto pos = topic.find(token);
  if (pos != std::string::npos)
    topic.replace(pos, token.size(), worldName);
  this->node.Subscribe(topic, &WecDashboardPrivate::OnJointState, this);
  this->node.Subscribe("/world/" + worldName + "/stats",
                       &WecDashboardPrivate::OnStats, this);
  gzmsg << "WecDashboard: mondo [" << worldName << "], telemetria connessa"
        << std::endl;
}

//////////////////////////////////////////////////
void WecDashboardPrivate::OnOdometry(const gz::msgs::Odometry &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  const double t = _msg.header().stamp().sec() +
                   1e-9 * _msg.header().stamp().nsec();
  this->simTime = t;
  const auto &p = _msg.pose().position();
  const auto &q = _msg.pose().orientation();
  const auto &lv = _msg.twist().linear();
  const auto &av = _msg.twist().angular();

  // RPY dal quaternione (convenzione ZYX, come telemetry.parsers)
  const double sinr = 2.0 * (q.w() * q.x() + q.y() * q.z());
  const double cosr = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  const double siny = 2.0 * (q.w() * q.z() + q.x() * q.y());
  const double cosy = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());

  this->values["hull_x_m"] = p.x();
  this->values["hull_y_m"] = p.y();
  this->values["hull_z_m"] = p.z();
  this->values["hull_roll_rad"] = std::atan2(sinr, cosr);
  this->values["hull_pitch_rad"] =
      std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
  this->values["hull_yaw_rad"] = std::atan2(siny, cosy);
  this->values["hull_vx_m_s"] = lv.x();
  this->values["hull_vy_m_s"] = lv.y();
  this->values["hull_vz_m_s"] = lv.z();
  this->values["hull_wx_rad_s"] = av.x();
  this->values["hull_wy_rad_s"] = av.y();
  this->values["hull_wz_rad_s"] = av.z();
  this->values["heave_eq_m"] = this->heaveEqZ;
  this->values["heave_rel_m"] = p.z() - this->heaveEqZ;
  this->PushSample(t);
}

//////////////////////////////////////////////////
void WecDashboardPrivate::OnJointState(const gz::msgs::Model &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  for (const auto &j : _msg.joint())
  {
    // axis1: posizione/velocita' del primo asse (prismatico PTO).
    // La forza non e' garantita dal JointStatePublisher: arriva da
    // /pto_state (WecDynamics) quando disponibile.
    this->values["pto_pos_m"] = j.axis1().position();
    this->values["pto_vel_m_s"] = j.axis1().velocity();
  }
}

//////////////////////////////////////////////////
void WecDashboardPrivate::OnPtoState(const gz::msgs::Float_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (_msg.data_size() < 5)
    return;
  const double pos = _msg.data(0);
  const double vel = _msg.data(1);
  const double force = _msg.data(2);
  const double power = _msg.data(3);
  const double energy = _msg.data(4);
  this->values["pto_pos_m"] = pos;
  this->values["pto_vel_m_s"] = vel;
  this->values["pto_force_N"] = force;
  this->values["pto_force_kN"] = force * 1e-3;
  this->values["pto_power_W"] = power;
  this->values["pto_power_kW"] = power * 1e-3;
  this->values["pto_energy_J"] = energy;
  this->values["pto_energy_kJ"] = energy * 1e-3;
}

//////////////////////////////////////////////////
void WecDashboardPrivate::OnAccel(const gz::msgs::Vector3d &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->values["hull_ax_m_s2"] = _msg.x();
  this->values["hull_ay_m_s2"] = _msg.y();
  this->values["hull_az_m_s2"] = _msg.z();
}

//////////////////////////////////////////////////
void WecDashboardPrivate::OnAngAccel(const gz::msgs::Vector3d &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->values["hull_wx_dot_rad_s2"] = _msg.x();
  this->values["hull_wy_dot_rad_s2"] = _msg.y();
  this->values["hull_wz_dot_rad_s2"] = _msg.z();
}

//////////////////////////////////////////////////
void WecDashboardPrivate::OnTension(const gz::msgs::Float_V &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  double total = 0.0;
  double tmax = 0.0;
  for (int i = 0; i < _msg.data_size(); ++i)
  {
    const double t = _msg.data(i);
    total += t;
    tmax = std::max(tmax, t);
  }
  this->values["fairten_total_N"] = total;
  this->values["fairten_max_N"] = tmax;
}

//////////////////////////////////////////////////
void WecDashboardPrivate::OnStats(const gz::msgs::WorldStatistics &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->values["sim_time_s"] = _msg.sim_time().sec() +
                               1e-9 * _msg.sim_time().nsec();
  this->values["real_time_factor"] = _msg.real_time_factor();
  this->values["iterations"] = static_cast<double>(_msg.iterations());
}

//////////////////////////////////////////////////
void WecDashboardPrivate::PushSample(double _time)
{
  if (this->paused)
    return;
  if (this->decimateHz > 0.0 && this->lastSampleTime >= 0.0 &&
      (_time - this->lastSampleTime) < (1.0 / this->decimateHz))
    return;
  this->lastSampleTime = _time;

  for (const auto &kv : this->values)
    this->series[kv.first].Push(_time, kv.second, this->historyS);

  double psdPeriod = 0.0;
  const QJsonObject dash = this->pipeline.value("dashboard").toObject();
  if (dash.contains("psd_refresh_hz") && dash.value("psd_refresh_hz").isDouble())
  {
    const double rate = dash.value("psd_refresh_hz").toDouble();
    if (rate > 0.0)
      psdPeriod = 1.0 / rate;
  }
  if (psdPeriod > 0.0 &&
      (this->lastPsdTime < 0.0 || (_time - this->lastPsdTime) >= psdPeriod))
    this->RefreshPsd();
}

//////////////////////////////////////////////////
void WecDashboardPrivate::RefreshCaches()
{
  std::lock_guard<std::mutex> lock(this->mutex);
  QJsonObject vals;
  for (const auto &kv : this->values)
    vals.insert(QString::fromStdString(kv.first), kv.second);
  this->valuesJsonCache = QString::fromUtf8(
      QJsonDocument(vals).toJson(QJsonDocument::Compact));

  QJsonObject seriesObj;
  QJsonArray tArr;
  if (!this->series.empty())
  {
    const auto &first = this->series.begin()->second;
    for (double t : first.t)
      tArr.append(t);
  }
  seriesObj.insert("t", tArr);
  QJsonObject seriesData;
  for (const auto &kv : this->series)
  {
    QJsonArray arr;
    for (double v : kv.second.v)
      arr.append(v);
    seriesData.insert(QString::fromStdString(kv.first), arr);
  }
  seriesObj.insert("series", seriesData);
  this->seriesJsonCache = QString::fromUtf8(
      QJsonDocument(seriesObj).toJson(QJsonDocument::Compact));

  QJsonObject psdObj;
  QJsonArray fArr;
  for (double f : this->psdFreqs)
    fArr.append(f);
  psdObj.insert("f_Hz", fArr);
  QJsonObject psdData;
  for (const auto &kv : this->psd)
  {
    QJsonArray arr;
    for (double v : kv.second)
      arr.append(v);
    psdData.insert(QString::fromStdString(kv.first), arr);
  }
  psdObj.insert("psd", psdData);
  this->psdJsonCache = QString::fromUtf8(
      QJsonDocument(psdObj).toJson(QJsonDocument::Compact));
}

//////////////////////////////////////////////////
void WecDashboardPrivate::RefreshPsd()
{
  this->lastPsdTime = this->simTime;
  this->psd.clear();
  this->psdFreqs.clear();

  // usa la serie heave_rel (o la prima disponibile) per la griglia frequenze
  auto collect = [&](const std::string &name) {
    auto it = this->series.find(name);
    if (it == this->series.end())
      return;
    const double t0 = this->simTime - this->psdWindowS;
    std::vector<double> t, x;
    for (size_t i = 0; i < it->second.t.size(); ++i)
    {
      if (it->second.t[i] >= t0)
      {
        t.push_back(it->second.t[i]);
        x.push_back(it->second.v[i]);
      }
    }
    std::vector<double> f, p;
    ComputePsd(t, x, this->fMinHz, this->fMaxHz, f, p);
    if (this->psdFreqs.empty())
      this->psdFreqs = f;
    this->psd[name] = p;
  };

  const QJsonArray charts = this->dashboard.value("charts").toArray();
  for (const auto &c : charts)
  {
    const QJsonObject ch = c.toObject();
    if (ch.value("kind").toString() != "psd")
      continue;
    for (const auto &s : ch.value("series").toArray())
      collect(s.toObject().value("source").toString().toStdString());
  }
}

//////////////////////////////////////////////////
WecDashboard::WecDashboard()
    : GuiSystem(), dataPtr(new WecDashboardPrivate)
{
}

WecDashboard::~WecDashboard() = default;

//////////////////////////////////////////////////
void WecDashboard::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "WEC Dashboard";

  if (!this->dataPtr->LoadConfiguration())
  {
    gzerr << "WecDashboard: configurazione incompleta\n";
    return;
  }
  this->dataPtr->Subscribe();
  this->dataPtr->configured = true;
  emit configChanged();

  bool standalone = false;
  if (_pluginElem)
  {
    if (const auto *elem = _pluginElem->FirstChildElement("standalone"))
      elem->QueryBoolText(&standalone);
  }
  auto *titleTimer = new QTimer(this);
  connect(titleTimer, &QTimer::timeout, this, [this, standalone, titleTimer]()
  {
    auto *item = this->PluginItem();
    if (!item)
    {
      titleTimer->stop();
      titleTimer->deleteLater();
      return;
    }
    auto *win = item->window();
    if (!win)
      return;
    if (standalone || win->title().isEmpty())
      win->setTitle("WEC Dashboard");
    titleTimer->stop();
    titleTimer->deleteLater();
  });
  titleTimer->start(200);

  auto *worldTimer = new QTimer(this);
  connect(worldTimer, &QTimer::timeout, this, [this, worldTimer]()
  {
    if (this->dataPtr->worldSubscribed)
    {
      worldTimer->stop();
      return;
    }
    this->dataPtr->DiscoverWorld();
  });
  worldTimer->start(500);
  this->dataPtr->DiscoverWorld();

  auto *uiTimer = new QTimer(this);
  connect(uiTimer, &QTimer::timeout, this, [this]()
  {
    this->dataPtr->RefreshCaches();
    emit valuesChanged();
    emit seriesChanged();
  });
  uiTimer->start(std::max(1, static_cast<int>(1000.0 / this->dataPtr->uiRefreshHz)));
}

//////////////////////////////////////////////////
QString WecDashboard::configJson() const
{
  QJsonObject out;
  out.insert("dashboard", this->dataPtr->dashboard);
  out.insert("pipeline", this->dataPtr->pipeline);
  out.insert("equilibrium", this->dataPtr->equilibrium);
  out.insert("title", this->title.c_str());
  return QString::fromUtf8(QJsonDocument(out).toJson(QJsonDocument::Compact));
}

QString WecDashboard::bandJson() const
{
  QJsonObject band = this->dataPtr->pipeline.value("wave_band").toObject();
  QJsonObject out;
  out.insert("period_min_s", band.value("period_min_s"));
  out.insert("period_max_s", band.value("period_max_s"));
  out.insert("f_min_Hz", this->dataPtr->fMinHz);
  out.insert("f_max_Hz", this->dataPtr->fMaxHz);
  return QString::fromUtf8(QJsonDocument(out).toJson(QJsonDocument::Compact));
}

QString WecDashboard::equilibriumJson() const
{
  return QString::fromUtf8(
      QJsonDocument(this->dataPtr->equilibrium).toJson(QJsonDocument::Compact));
}

QString WecDashboard::seriesJson() const
{
  return this->dataPtr->seriesJsonCache;
}

QString WecDashboard::valuesJson() const
{
  return this->dataPtr->valuesJsonCache;
}

QString WecDashboard::psdJson() const
{
  return this->dataPtr->psdJsonCache;
}

bool WecDashboard::live() const
{
  return this->dataPtr->configured && !this->dataPtr->paused;
}

//////////////////////////////////////////////////
void WecDashboard::clearHistory()
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    for (auto &kv : this->dataPtr->series)
      kv.second.Clear();
    this->dataPtr->psd.clear();
    this->dataPtr->psdFreqs.clear();
  }
  this->dataPtr->RefreshCaches();
  emit seriesChanged();
}

void WecDashboard::setPaused(bool _paused)
{
  this->dataPtr->paused = _paused;
  emit valuesChanged();
}

void WecDashboard::setWindowSeconds(double _seconds)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (_seconds > 0.0)
    this->dataPtr->windowS = _seconds;
}

}  // namespace wec

GZ_ADD_PLUGIN(wec::WecDashboard, gz::gui::Plugin)
