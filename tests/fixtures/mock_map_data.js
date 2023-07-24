// ==========================
//   FAKE MAP DATA FOR TEST
// ==========================

// Data Source, which may be from file or ROS msg. topic

const cellTypes = {
  "RackDetection": {
    "label": " ⬛ Rack",
    "color": {
      "bgColor": "rgba(0, 0, 0, 0.5)",
      "borderColor": "lightgray",
      "color": "lightgray",
      "focusColor": "black",
      "tooltipColor": "black"
    },
    "value": "rack",
    "width": "2",
    "length": "6",
    "alias": [
      "rack"
    ]
  },
  "ChargerDetection": {
    "label": " 🟩 Charger",
    "color": {
      "bgColor": "rgba(100, 160, 71, 0.7)",
      "borderColor": "#79b574",
      "color": "#c1e6cc",
      "focusColor": "rgba(100, 160, 71, 0.9)",
      "tooltipColor": "#79b574"
    },
    "value": "charger",
    "width": "2",
    "length": "2",
    "alias": [
      "charger"
    ]
  },
  "QRcode": {
    "label": " ⬜ QR code",
    "color": {
      "bgColor": "rgba(255, 255, 255, 0.5)",
      "borderColor": "gray",
      "color": "gray",
      "focusColor": "rgba(0, 0, 0, 0.2)",
      "tooltipColor": "gray"
    },
    "value": "qrcode",
    "width": "1",
    "length": "1",
    "alias": [
      "qrcode"
    ]
  },
  "Position": {
    "label": " 🟦 Position",
    "color": {
      "bgColor": "rgba(43, 118, 183, 0.5)",
      "borderColor": "rgba(45, 83, 107, 0.5)",
      "color": "#d1e3f0",
      "focusColor": "rgba(45, 83, 107, 0.5)",
      "tooltipColor": "#2b76b7"
    },
    "value": "position",
    "width": "1",
    "length": "1",
    "alias": [
      "position"
    ]
  }
};

const cellCollection = {
  "area1": [
    {
      "cell_coordinate": [
        4.8571,
        2.6656
      ],
      "cell_id": "cell_1",
      "direction": "forward",
      "load": "empty",
      "status": "empty",
      "type": "QRcode",
      "width": 0,
      "length": 0
    },
    {
      "cell_coordinate": [
        4.7429,
        4.6585
      ],
      "cell_id": "cell_2",
      "direction": "forward",
      "load": "empty",
      "status": "empty",
      "type": "RackDetection",
      "width": 0,
      "length": 0
    },
    {
      "cell_coordinate": [
        5.85,
        3.7942
      ],
      "cell_id": "cell_3",
      "direction": "forward",
      "load": "empty",
      "status": "empty",
      "type": "ChargerDetection",
      "width": 0,
      "length": 0
    },
    {
      "cell_coordinate": [
        3.6929,
        3.6371
      ],
      "cell_id": "cell_4",
      "direction": "forward",
      "load": "empty",
      "status": "empty",
      "type": "Position",
      "width": 0,
      "length": 0
    }
  ]
}

module.exports = {
  cellTypes,
  cellCollection
}
