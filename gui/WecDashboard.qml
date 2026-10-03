import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12
import "qrc:/WecDashboard"

// WecDashboard -- root del plugin GUI.
// Tema, cards, charts, bande e soglie arrivano da config/dashboard.json e
// config/pipeline.json (espone _WecDashboard.configJson / bandJson / ...).
// NESSUN valore di layout/stato e' hard-coded qui.
Rectangle {
    id: root

    property var cfg: ({})
    property var dash: cfg.dashboard || ({})
    property var theme: dash.theme || ({})
    property var cards: dash.cards || []
    property var charts: dash.charts || []
    property var band: ({})
    property var values: ({})
    property var series: ({ t: [], series: {} })
    property var psd: ({ f_Hz: [], psd: {} })
    property var alarms: dash.alarms || ({})

    function colorOf(name) {
        return theme[name] !== undefined ? theme[name] : "#3dd6c4"
    }

    function fmt(v, decimals) {
        if (v === undefined || v === null || !isFinite(v)) return "--"
        return Number(v).toFixed(decimals !== undefined ? decimals : 2)
    }

    function cardValue(card) {
        var v = values[card.source]
        if (v === undefined) return "--"
        if (card.to_display_unit !== undefined) v = v * card.to_display_unit
        return fmt(v, card.decimals)
    }

    function cardAlarm(card) {
        if (!alarms) return false
        if (card.source === "heave_rel_m" && alarms.heave_rel_abs_m !== undefined)
            return Math.abs(values["heave_rel_m"] || 0) > alarms.heave_rel_abs_m
        if (card.source === "fairten_total_N" && alarms.fairten_total_N !== undefined)
            return (values["fairten_total_N"] || 0) > alarms.fairten_total_N
        if (card.source === "real_time_factor" && alarms.real_time_factor_low !== undefined)
            return (values["real_time_factor"] || 0) < alarms.real_time_factor_low
        return false
    }

    function parseJson(text) {
        try { return JSON.parse(text) } catch (e) { return ({}) }
    }

    color: colorOf("bg")
    implicitWidth: 420
    implicitHeight: 720

    Component.onCompleted: {
        cfg = parseJson(_WecDashboard.configJson)
        band = parseJson(_WecDashboard.bandJson)
        band.overlay = dash.band_overlay === true
        band.color = Qt.rgba(0.94, 0.65, 0.25, 0.10)
    }

    Connections {
        target: _WecDashboard
        function onConfigChanged() {
            cfg = parseJson(_WecDashboard.configJson)
            band = parseJson(_WecDashboard.bandJson)
            band.overlay = dash.band_overlay === true
            band.color = Qt.rgba(0.94, 0.65, 0.25, 0.10)
        }
        function onValuesChanged() {
            values = parseJson(_WecDashboard.valuesJson)
        }
        function onSeriesChanged() {
            series = parseJson(_WecDashboard.seriesJson)
            psd = parseJson(_WecDashboard.psdJson)
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 8

        // ---- header --------------------------------------------------
        RowLayout {
            Layout.fillWidth: true
            spacing: 8

            Text {
                text: _WecDashboard.title || "WEC DASHBOARD"
                color: colorOf("accent")
                font.pixelSize: 14
                font.bold: true
                Layout.fillWidth: true
            }

            Rectangle {
                width: 10; height: 10; radius: 5
                color: _WecDashboard.live ? colorOf("accent") : colorOf("warn")
            }

            Button {
                text: _WecDashboard.live ? qsTr("Pausa") : qsTr("Riprendi")
                onClicked: _WecDashboard.setPaused(_WecDashboard.live)
            }
            Button {
                text: qsTr("Pulisci")
                onClicked: _WecDashboard.clearHistory()
            }
        }

        // ---- cards ---------------------------------------------------
        GridLayout {
            Layout.fillWidth: true
            columns: 5
            rowSpacing: 6
            columnSpacing: 6

            Repeater {
                model: root.cards
                delegate: StatCard {
                    Layout.fillWidth: true
                    label: modelData.label
                    unit: modelData.unit
                    value: root.cardValue(modelData)
                    accent: root.colorOf("accent")
                    bg: root.colorOf("card")
                    fg: root.colorOf("text")
                    muted: root.colorOf("muted")
                    alarm: root.cardAlarm(modelData)
                }
            }
        }

        // ---- charts --------------------------------------------------
        Flickable {
            Layout.fillWidth: true
            Layout.fillHeight: true
            contentHeight: chartColumn.implicitHeight
            clip: true

            ColumnLayout {
                id: chartColumn
                width: parent.width
                spacing: 8

                Repeater {
                    model: root.charts
                    delegate: TimeSeriesChart {
                        Layout.fillWidth: true
                        Layout.preferredHeight: modelData.kind === "psd" ? 160 : 140
                        title: modelData.title
                        yUnit: modelData.y_unit || ""
                        y2Unit: modelData.y2_unit || ""
                        kind: modelData.kind || "time"
                        xScale: modelData.x_scale || "linear"
                        theme: root.theme
                        series: modelData.series || []
                        band: modelData.kind === "psd" ? null : root.band
                        windowSeconds: root.dash.window_s || 0
                        dataBundle: modelData.kind === "psd"
                            ? root.psd
                            : root.series
                    }
                }
            }

            ScrollBar.vertical: ScrollBar {}
        }

        // ---- footer banda ---------------------------------------------
        Text {
            Layout.fillWidth: true
            color: colorOf("muted")
            font.pixelSize: 10
            text: band.f_min_Hz !== undefined
                ? qsTr("Banda onde T=") + band.period_min_s + ".." + band.period_max_s
                  + qsTr(" s  |  f=") + Number(band.f_min_Hz).toFixed(3) + ".."
                  + Number(band.f_max_Hz).toFixed(2) + " Hz"
                : ""
        }
    }
}
