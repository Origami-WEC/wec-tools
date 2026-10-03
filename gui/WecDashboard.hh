// Copyright (C) 2026  maritime_ws
//
// WecDashboard -- dashboard live in Gazebo per il dispositivo WEC.
//
// Tema, cards, grafici, soglie e banda non sono hard-coded: vengono caricati
// da config/dashboard.json + config/pipeline.json + logs/equilibrium.json
// (percorsi dichiarati negli stessi config). Vedi WecDashboard.hh.

#ifndef WEC_TOOLS_WEC_DASHBOARD_HH_
#define WEC_TOOLS_WEC_DASHBOARD_HH_

#include <memory>

#include <gz/sim/gui/GuiSystem.hh>

namespace wec
{

class WecDashboardPrivate;

class WecDashboard : public gz::sim::GuiSystem
{
  Q_OBJECT

  /// Payload JSON di configurazione (tema, cards, charts, alarms).
  Q_PROPERTY(QString configJson READ configJson NOTIFY configChanged)
  /// Banda frequenza come JSON {f_min_Hz, f_max_Hz, period_min_s, period_max_s}.
  Q_PROPERTY(QString bandJson READ bandJson NOTIFY configChanged)
  /// Equilibrio come JSON (waterline, spawn_z, cob, com, gm).
  Q_PROPERTY(QString equilibriumJson READ equilibriumJson NOTIFY configChanged)
  /// Serie storiche come JSON: {t:[...], series:{name:[...]}}.
  Q_PROPERTY(QString seriesJson READ seriesJson NOTIFY seriesChanged)
  /// Valori istantanei come JSON: {source: value}.
  Q_PROPERTY(QString valuesJson READ valuesJson NOTIFY valuesChanged)
  /// Spettro come JSON: {f_Hz:[...], psd:{name:[...]}}.
  Q_PROPERTY(QString psdJson READ psdJson NOTIFY seriesChanged)
  Q_PROPERTY(bool live READ live NOTIFY valuesChanged)

  public: explicit WecDashboard();
  public: ~WecDashboard() override;

  public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  public: QString configJson() const;
  public: QString bandJson() const;
  public: QString equilibriumJson() const;
  public: QString seriesJson() const;
  public: QString valuesJson() const;
  public: QString psdJson() const;
  public: bool live() const;

  /// Comandi dalla UI.
  public slots: void clearHistory();
  public slots: void setPaused(bool _paused);
  public slots: void setWindowSeconds(double _seconds);

  signals: void configChanged();
  signals: void seriesChanged();
  signals: void valuesChanged();

  private: std::unique_ptr<WecDashboardPrivate> dataPtr;
};

}  // namespace wec

#endif
