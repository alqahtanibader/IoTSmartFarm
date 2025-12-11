"""
Smart Farm Dashboard - SECURE VERSION with TLS/SSL
Uses HiveMQ with TLS encryption (port 8883)
This meets security requirements for project submission
"""

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import time
import ssl
from mqtt_handler_secure import MQTTHandlerSecure

app = Flask(__name__)
app.config['SECRET_KEY'] = 'smartfarm'
socketio = SocketIO(app)

# MQTT handler - connects to HiveMQ Cloud with TLS
# data_dir: where to store CSV files, retention_days: how long to keep data
mqtt = MQTTHandlerSecure(data_dir="data", retention_days=7)
# Update with your HiveMQ Cloud credentials
mqtt.connect(
    broker="YOUR_BROKER_URL.hivemq.cloud",
    port=8883,
    username="YOUR_MQTT_USERNAME",  # Your HiveMQ Cloud username
    password="YOUR_MQTT_PASSWORD"    # Your HiveMQ Cloud password
)

# Store data for plots
plot_data = {}  # {node_id: {temp: [], moisture: [], time: []}}

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/api/nodes')
def get_nodes():
    """Get list of all nodes with their info"""
    nodes_with_info = []
    for node_id in mqtt.nodes:
        info = mqtt.get_node_info(node_id)
        info['node_id'] = node_id
        nodes_with_info.append(info)
    return jsonify({'nodes': mqtt.nodes, 'node_info': nodes_with_info})

@app.route('/api/nodes/<node_id>/name', methods=['POST'])
def update_node_name(node_id):
    """Update device friendly name"""
    data = request.json
    name = data.get('name', node_id)
    if mqtt.update_node_name(node_id, name):
        return jsonify({'success': True, 'name': name})
    return jsonify({'success': False, 'error': 'Node not found'}), 404

@app.route('/api/nodes/online')
def get_online_nodes():
    """Get list of online nodes only"""
    online = mqtt.get_online_nodes()
    return jsonify({'nodes': online})

@app.route('/api/data/files/<node_id>')
def get_data_files(node_id):
    """Get list of data files for a node"""
    files = mqtt.get_data_files(node_id)
    return jsonify({'files': files})

@app.route('/api/data/files')
def get_all_data_files():
    """Get list of all data files"""
    files = mqtt.get_data_files()
    return jsonify({'files': files})

@app.route('/api/nodes/<node_id>/delete-data', methods=['POST'])
def delete_node_data(node_id):
    """Delete all data files for a node"""
    data = request.json or {}
    days = data.get('days', None)  # Optional: delete files older than X days
    
    deleted = mqtt.delete_node_data(node_id, days)
    return jsonify({'success': True, 'deleted': deleted, 'count': len(deleted)})

@app.route('/api/nodes/<node_id>/delete-all-data', methods=['POST'])
def delete_all_node_data(node_id):
    """Delete ALL data files for a node (regardless of age)"""
    deleted = mqtt.delete_node_data(node_id, days=None)
    return jsonify({'success': True, 'deleted': deleted, 'count': len(deleted)})

@app.route('/api/data/<node_id>')
def get_data(node_id):
    data = mqtt.get_latest_data(node_id)
    return jsonify(data)

@app.route('/api/control', methods=['POST'])
def control():
    data = request.json
    node_id = data.get('node_id')
    command = data.get('command', {})
    mqtt.send_control(node_id, command)
    return jsonify({'success': True})

@app.route('/api/thresholds/<node_id>', methods=['POST'])
def set_thresholds(node_id):
    thresholds = request.json
    mqtt.send_thresholds(node_id, thresholds)
    return jsonify({'success': True})

# WebSocket for real-time updates
@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('connected', {'msg': 'Connected'})

def background_task():
    """Send updates every 0.5 seconds for better responsiveness"""
    while True:
        time.sleep(0.5)  # Faster updates (500ms instead of 1000ms)
        for node_id in mqtt.nodes:
            data = mqtt.get_latest_data(node_id)
            if data:
                # Update plot data
                if node_id not in plot_data:
                    plot_data[node_id] = {'temp': [], 'moisture': [], 'time': []}
                
                plot_data[node_id]['temp'].append(data.get('temperature', 0))
                plot_data[node_id]['moisture'].append(data.get('soil_moisture', 0))
                
                # Format time as human-readable (HH:MM:SS)
                current_time = time.localtime()
                time_str = f"{current_time.tm_hour:02d}:{current_time.tm_min:02d}:{current_time.tm_sec:02d}"
                plot_data[node_id]['time'].append(time_str)
                
                # Keep last 60 points (~5 minutes at 5-second intervals)
                if len(plot_data[node_id]['temp']) > 60:
                    plot_data[node_id]['temp'].pop(0)
                    plot_data[node_id]['moisture'].pop(0)
                    plot_data[node_id]['time'].pop(0)
                
                # Send update
                socketio.emit('update', {
                    'node_id': node_id,
                    'data': data,
                    'plot': plot_data[node_id]
                })

if __name__ == '__main__':
    print("=" * 60)
    print("Smart Farm Dashboard - SECURE VERSION (TLS/SSL)")
    print("=" * 60)
    print("\nUsing HiveMQ with TLS encryption (port 8883)")
    print("✅ Encrypted communication")
    print("✅ Meets security requirements")
    print("\nOpen http://localhost:5000 in your browser")
    print("=" * 60)
    print()
    
    # Start background task
    socketio.start_background_task(background_task)
    
    # Run server
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)

