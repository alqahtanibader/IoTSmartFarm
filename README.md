# IoT Smart Farm

A secure IoT-based smart farm monitoring and control system using ESP32 and MQTT over TLS/SSL.

## 🌱 Features

- **Real-time Monitoring**: Temperature and soil moisture sensors
- **Automatic Control**: Fan and water pump activation based on configurable thresholds
- **Secure Communication**: MQTT over TLS/SSL (port 8883) with HiveMQ Cloud
- **Web Dashboard**: Real-time visualization with Plotly.js charts
- **Multi-Node Support**: Scalable architecture supporting multiple sensor nodes
- **Edge Computing**: Local auto-control logic runs on ESP32 for low latency

## 🏗️ Architecture

```
┌─────────────┐     MQTT/TLS      ┌─────────────┐     MQTT/TLS      ┌─────────────┐
│   ESP32     │ ───────────────►  │   HiveMQ    │  ◄─────────────── │   Flask     │
│   Node      │     Port 8883     │   Cloud     │     Port 8883     │   Server    │
└─────────────┘                   └─────────────┘                   └─────────────┘
      │                                                                    │
      │                                                                    │
  ┌───┴───┐                                                          ┌─────┴─────┐
  │Sensors│                                                          │ Dashboard │
  │ BMP280│                                                          │  Browser  │
  │ Soil  │                                                          │ WebSocket │
  └───────┘                                                          └───────────┘
```

## 📁 Project Structure

```
IoTSmartFarm/
├── esp32_node/
│   └── esp32_node_secure.ino    # ESP32 Arduino code
├── server/
│   ├── dashboard_secure.py      # Flask web server
│   ├── mqtt_handler_secure.py   # MQTT client handler
│   ├── requirements.txt         # Python dependencies
│   └── templates/
│       └── dashboard.html       # Web dashboard UI
├── QUICK_START.txt              # Setup guide
└── README.md
```

## 🔧 Hardware Requirements

- ESP32-S3-DevKit-C-1 (or compatible ESP32 board)
- BMP280 Temperature Sensor
- Capacitive Soil Moisture Sensor
- 5V DC Fan
- DC Water Pump
- 2x IRFZ44N MOSFET (for actuator control)
- Resistors: 2x 330Ω, 2x 10kΩ
- External 5V Power Supply

## 🚀 Quick Start

### 1. HiveMQ Cloud Setup
1. Create free account at [HiveMQ Cloud](https://www.hivemq.com/cloud/)
2. Create a cluster and note the broker URL
3. Create credentials (username/password)

### 2. ESP32 Setup
1. Open `esp32_node/esp32_node_secure.ino` in Arduino IDE
2. Install required libraries:
   - PubSubClient
   - Adafruit BMP280
   - ArduinoJson
3. Update WiFi and MQTT credentials in the code
4. Upload to ESP32

### 3. Server Setup
```bash
cd server
pip install -r requirements.txt
# Update credentials in dashboard_secure.py
python dashboard_secure.py
```

### 4. Access Dashboard
Open `http://localhost:5000` in your browser

## 🔐 Security Features

- **TLS/SSL Encryption**: All MQTT traffic encrypted (port 8883)
- **Authentication**: Username/password required for broker connection
- **Topic Isolation**: Each node has unique topic namespace

## 📊 Dashboard Features

- Real-time temperature and moisture display
- Interactive charts (last 5 minutes of data)
- Manual/Auto control mode toggle
- Adjustable thresholds
- Multi-node device selection

## 🛠️ Configuration

### ESP32 Configuration (esp32_node_secure.ino)
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "YOUR_BROKER.hivemq.cloud";
const char* mqtt_username = "YOUR_USERNAME";
const char* mqtt_password = "YOUR_PASSWORD";
const char* node_id = "node001";
```

### Server Configuration (dashboard_secure.py)
```python
mqtt.connect(
    broker="YOUR_BROKER.hivemq.cloud",
    port=8883,
    username="YOUR_USERNAME",
    password="YOUR_PASSWORD"
)
```

## 📝 License

This project was developed for educational purposes as part of COE 550 - IoT Systems course.

## 👤 Author

Bader Al Qahtani
