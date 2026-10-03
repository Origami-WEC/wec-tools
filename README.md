# wec-tools — gz-sim plugins and dashboard for the WEC

Two Gazebo Sim plugins used by the WEC simulation pipeline:

| Component | Type | What |
|---|---|---|
| `system/WecDynamics` | system plugin | Reads accelerations and PTO state from the simulated model |
| `gui/WecDashboard` | GUI plugin | QML dashboard (`WecDashboard.qml`, `StatCard.qml`, `TimeSeriesChart.qml`) rendering the live telemetry |

## Build

Requires `gz-sim` (with the `gui` component), `gz-gui`, `gz-transport`,
`gz-msgs`, `gz-math`, `gz-plugin` and **Qt6** (Core, Qml, Quick).

```bash
cmake -B build -S .
cmake --build build
```

## Part of

The Origami simulation stack — loaded by the world/model in `maritime_ws`.
