import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12

// Card numero grande. Label/unita'/colori arrivano da config/dashboard.json
Rectangle {
    id: root

    property string label: ""
    property string unit: ""
    property string value: "--"
    property string accent: "#3dd6c4"
    property string bg: "#1a2332"
    property string fg: "#e6edf3"
    property string muted: "#8b9cb3"
    property bool alarm: false

    radius: 8
    color: bg
    border.color: alarm ? "#f07178" : Qt.darker(bg, 1.2)
    border.width: alarm ? 2 : 1
    implicitWidth: 110
    implicitHeight: 64

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 8
        spacing: 2

        RowLayout {
            Layout.fillWidth: true
            spacing: 4
            Text {
                text: root.label
                color: root.muted
                font.pixelSize: 11
                font.bold: true
                Layout.fillWidth: true
                elide: Text.ElideRight
            }
            Text {
                text: root.unit
                color: root.muted
                font.pixelSize: 10
            }
        }

        Text {
            text: root.value
            color: root.alarm ? "#f07178" : root.fg
            font.pixelSize: 18
            font.bold: true
            font.family: "Menlo, monospace"
            Layout.fillWidth: true
        }
    }

    Rectangle {
        anchors.left: parent.left
        anchors.leftMargin: 0
        anchors.verticalCenter: parent.verticalCenter
        width: 3
        height: parent.height * 0.55
        radius: 1
        color: root.alarm ? "#f07178" : root.accent
    }
}
