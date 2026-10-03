import QtQuick 2.12

// Grafico a serie multiple su Canvas. Nessun colore/etichetta hard-coded:
// `theme`, `series` e `band` arrivano da config/dashboard.json.
// kind: "time" (x = tempo) | "psd" (x = frequenza, opz. log)
Item {
    id: root

    property string title: ""
    property string yUnit: ""
    property string y2Unit: ""
    property string kind: "time"
    property string xScale: "linear"
    property var theme: ({})
    property var series: []
    property var band: null
    property var dataBundle: ({ t: [], series: {}, f_Hz: [], psd: {} })
    property real windowSeconds: 30.0

    property color cBg: theme && theme.card ? theme.card : "#1a2332"
    property color cFg: theme && theme.text ? theme.text : "#e6edf3"
    property color cMuted: theme && theme.muted ? theme.muted : "#8b9cb3"
    property color cGrid: theme && theme.grid ? theme.grid : "#2a3544"

    function themeColor(name) {
        if (!theme) return "#3dd6c4"
        return theme[name] !== undefined ? theme[name] : "#3dd6c4"
    }

    function seriesColor(s) {
        return themeColor(s.color || "accent")
    }

    implicitWidth: 320
    implicitHeight: 150

    Rectangle {
        anchors.fill: parent
        radius: 8
        color: root.cBg
        border.color: Qt.darker(root.cBg, 1.2)
        border.width: 1
    }

    Text {
        id: titleLabel
        anchors.top: parent.top
        anchors.left: parent.left
        anchors.margins: 8
        text: root.title
        color: root.cFg
        font.pixelSize: 11
        font.bold: true
    }

    Canvas {
        id: canvas
        anchors.top: titleLabel.bottom
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: legend.top
        anchors.margins: 6

        onPaint: {
            var ctx = getContext("2d")
            ctx.reset()
            var w = width, h = height
            if (w <= 0 || h <= 0) return

            // griglia
            ctx.strokeStyle = root.cGrid
            ctx.lineWidth = 1
            var nGrid = 4
            for (var g = 0; g <= nGrid; g++) {
                var y = (h - 1) * g / nGrid
                ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke()
            }

            var isPsd = (root.kind === "psd")
            var xs = isPsd ? (dataBundle.f_Hz || []) : (dataBundle.t || [])
            var n = xs.length
            if (n < 2) {
                ctx.fillStyle = root.cMuted
                ctx.font = "11px sans-serif"
                ctx.fillText("in attesa di dati...", 8, h / 2)
                return
            }

            var x0 = xs[0], x1 = xs[n - 1]
            if (!isPsd && root.windowSeconds > 0) {
                x0 = Math.max(x0, x1 - root.windowSeconds)
            }
            var xSpan = Math.max(x1 - x0, 1e-9)

            // scala y su tutte le serie asse y (e y2 separata se presente)
            var yMin = Infinity, yMax = -Infinity, y2Min = Infinity, y2Max = -Infinity
            for (var s = 0; s < root.series.length; s++) {
                var spec = root.series[s]
                var arr = isPsd ? (dataBundle.psd || {})[spec.source] || []
                                : (dataBundle.series || {})[spec.source] || []
                var useY2 = (spec.axis === "y2")
                for (var i = 0; i < arr.length; i++) {
                    var v = arr[i]
                    if (!isFinite(v)) continue
                    if (spec.to_display_unit !== undefined) v = v * spec.to_display_unit
                    if (useY2) {
                        if (v < y2Min) y2Min = v
                        if (v > y2Max) y2Max = v
                    } else {
                        if (v < yMin) yMin = v
                        if (v > yMax) yMax = v
                    }
                }
            }
            if (!isFinite(yMin)) { yMin = -1; yMax = 1 }
            if (yMax - yMin < 1e-9) { yMin -= 1; yMax += 1 }
            if (!isFinite(y2Min)) { y2Min = yMin; y2Max = yMax }
            if (y2Max - y2Min < 1e-9) { y2Min -= 1; y2Max += 1 }
            if (isPsd) { yMin = 0 }

            function mapX(xv) {
                if (root.xScale === "log" && xv > 0 && x0 > 0) {
                    var lx0 = Math.log(x0), lx1 = Math.log(x1)
                    return (Math.log(xv) - lx0) / Math.max(lx1 - lx0, 1e-9) * (w - 2) + 1
                }
                return (xv - x0) / xSpan * (w - 2) + 1
            }
            function mapY(vv, useY2) {
                var lo = useY2 ? y2Min : yMin
                var hi = useY2 ? y2Max : yMax
                return h - 1 - (vv - lo) / Math.max(hi - lo, 1e-9) * (h - 2)
            }

            // marker banda onde
            if (!isPsd && root.band && root.band.overlay) {
                ctx.fillStyle = root.band.color || "rgba(240,167,63,0.08)"
                ctx.fillRect(0, 0, w, h)
            }

            // serie
            for (var s2 = 0; s2 < root.series.length; s2++) {
                var sp = root.series[s2]
                var data = isPsd ? (dataBundle.psd || {})[sp.source] || []
                                 : (dataBundle.series || {})[sp.source] || []
                ctx.strokeStyle = root.seriesColor(sp)
                ctx.lineWidth = sp.style === "dashed" ? 1 : 1.6
                ctx.setLineDash(sp.style === "dashed" ? [4, 3] : [])
                ctx.beginPath()
                var started = false
                for (var k = 0; k < data.length; k++) {
                    var xv = xs[Math.min(k, n - 1)]
                    if (xv < x0) continue
                    var yv = data[k]
                    if (!isFinite(yv)) { started = false; continue }
                    if (sp.to_display_unit !== undefined) yv = yv * sp.to_display_unit
                    var px = mapX(xv), py = mapY(yv, sp.axis === "y2")
                    if (!started) { ctx.moveTo(px, py); started = true }
                    else ctx.lineTo(px, py)
                }
                ctx.stroke()
                ctx.setLineDash([])
            }
        }

        Connections {
            target: root
            function onDataBundleChanged() { canvas.requestPaint() }
            function onWidthChanged() { canvas.requestPaint() }
            function onHeightChanged() { canvas.requestPaint() }
        }
        Timer {
            interval: 50; running: true; repeat: true
            onTriggered: canvas.requestPaint()
        }
    }

    Row {
        id: legend
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 6
        height: 14
        spacing: 10

        Repeater {
            model: root.series
            delegate: Row {
                spacing: 4
                Rectangle {
                    width: 8; height: 3; radius: 1
                    anchors.verticalCenter: parent.verticalCenter
                    color: root.seriesColor(modelData)
                }
                Text {
                    text: modelData.label || modelData.source
                    color: root.cMuted
                    font.pixelSize: 9
                }
            }
        }

        Item { width: 8; height: 1 }
        Text {
            visible: root.yUnit !== ""
            text: root.yUnit + (root.y2Unit !== "" ? " / " + root.y2Unit : "")
            color: root.cMuted
            font.pixelSize: 9
            anchors.verticalCenter: parent.verticalCenter
        }
    }
}
