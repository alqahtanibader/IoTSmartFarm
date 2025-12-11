"""
MQTT Handler for HiveMQ with TLS/SSL Encryption
Secure version - uses port 8883 with TLS
"""

import json
import paho.mqtt.client as mqtt
import ssl
import time
import csv
import os
from datetime import datetime, timedelta
from typing import Dict, List

class MQTTHandlerSecure:
    def __init__(self, data_dir="data", retention_days=7):
        self.sensor_data = {}  # {node_id: latest_data}
        self.nodes = []
        self.node_info = {}  # {node_id: {name, last_seen, status}}
        self.offline_timeout = 300  # 5 minutes - device considered offline
        
        # Data logging settings
        self.data_dir = data_dir
        self.retention_days = retention_days
        
        # Create data directory if it doesn't exist
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
            print(f"Created data directory: {self.data_dir}")
        
        # Initialize CSV files for each node
        self.csv_files = {}  # {node_id: file_handle}
        
        # Clean up old data on startup
        self._cleanup_old_data()
        
        # Connect to MQTT with secure client
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        # HiveMQ Cloud requires username/password
        # Set these in connect() method
        
    def connect(self, broker="YOUR_BROKER_URL.hivemq.cloud", 
                port=8883, username=None, password=None):
        """Connect to HiveMQ Cloud broker with TLS/SSL encryption"""
        try:
            # Set username/password if provided
            if username and password:
                self.client.username_pw_set(username, password)
            
            # Enable TLS
            self.client.tls_set(cert_reqs=ssl.CERT_NONE)  # Skip certificate validation (for testing)
            # For production, use: self.client.tls_set(ca_certs="path/to/ca.crt")
            
            self.client.connect(broker, port, 60)
            self.client.loop_start()
            print(f"Connected to HiveMQ Cloud broker with TLS at {broker}:{port}")
            print("✅ Communication is encrypted")
            if username:
                print("✅ Authentication enabled")
        except Exception as e:
            print(f"MQTT connection error: {e}")
            print("\nMake sure you have internet connection!")
    
    def _on_connect(self, client, userdata, flags, rc):
        """Called when connected to broker"""
        if rc == 0:
            print("MQTT connected to HiveMQ (SECURE TLS)!")
            # Subscribe to all sensor topics
            client.subscribe("farm/+/sensors")
        else:
            print(f"MQTT connection failed: {rc}")
    
    def _on_message(self, client, userdata, msg):
        """Called when message received"""
        try:
            topic = msg.topic
            data = json.loads(msg.payload.decode())
            
            # Extract node ID from topic (farm/node001/sensors)
            node_id = topic.split('/')[1]
            
            # Store data
            self.sensor_data[node_id] = data
            
            # Log to CSV file
            self._log_to_csv(node_id, data)
            
            # Update node list and info
            if node_id not in self.nodes:
                self.nodes.append(node_id)
                self.node_info[node_id] = {
                    'name': node_id,  # Default name is node_id
                    'last_seen': time.time(),
                    'status': 'online'
                }
                print(f"New node detected: {node_id}")
            else:
                # Update last seen time
                if node_id in self.node_info:
                    self.node_info[node_id]['last_seen'] = time.time()
                    self.node_info[node_id]['status'] = 'online'
                
        except Exception as e:
            print(f"Error processing message: {e}")
    
    def get_latest_data(self, node_id=None):
        """Get latest sensor data"""
        if node_id:
            return self.sensor_data.get(node_id, {})
        return self.sensor_data.copy()
    
    def send_control(self, node_id, command):
        """Send control command to node"""
        topic = f"farm/{node_id}/control"
        payload = json.dumps(command)
        self.client.publish(topic, payload)
        print(f"Sent control to {node_id}: {command}")
    
    def send_thresholds(self, node_id, thresholds):
        """Send threshold updates to node"""
        topic = f"farm/{node_id}/thresholds"
        payload = json.dumps(thresholds)
        self.client.publish(topic, payload)
        print(f"Sent thresholds to {node_id}: {thresholds}")
    
    def get_node_info(self, node_id=None):
        """Get device information"""
        if node_id:
            return self.node_info.get(node_id, {})
        return self.node_info.copy()
    
    def update_node_name(self, node_id, name):
        """Update device friendly name"""
        if node_id in self.node_info:
            self.node_info[node_id]['name'] = name
            print(f"Updated device name: {node_id} -> {name}")
            return True
        return False
    
    def get_online_nodes(self):
        """Get list of online nodes (nodes that sent data recently)"""
        current_time = time.time()
        online_nodes = []
        
        for node_id in self.nodes:
            if node_id in self.node_info:
                last_seen = self.node_info[node_id].get('last_seen', 0)
                time_since_seen = current_time - last_seen
                
                if time_since_seen < self.offline_timeout:
                    self.node_info[node_id]['status'] = 'online'
                    online_nodes.append(node_id)
                else:
                    self.node_info[node_id]['status'] = 'offline'
        
        return online_nodes
    
    def _log_to_csv(self, node_id, data):
        """Log sensor data to CSV file"""
        try:
            # Get current date for filename (one file per day per node)
            date_str = datetime.now().strftime("%Y-%m-%d")
            filename = os.path.join(self.data_dir, f"{node_id}_{date_str}.csv")
            
            # Check if file exists to determine if we need headers
            file_exists = os.path.exists(filename)
            
            # Open file in append mode
            with open(filename, 'a', newline='') as csvfile:
                fieldnames = [
                    'timestamp', 'datetime', 'node_id', 'temperature',
                    'soil_moisture', 'soil_moisture_raw', 'fan', 'pump', 'auto_mode'
                ]
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                # Write header if new file
                if not file_exists:
                    writer.writeheader()
                
                # Prepare data row
                timestamp = time.time()
                datetime_str = datetime.fromtimestamp(timestamp).strftime("%Y-%m-%d %H:%M:%S")
                
                row = {
                    'timestamp': f"{timestamp:.3f}",
                    'datetime': datetime_str,
                    'node_id': node_id,
                    'temperature': f"{data.get('temperature', 0):.2f}",
                    'soil_moisture': f"{data.get('soil_moisture', 0):.2f}",
                    'soil_moisture_raw': data.get('soil_moisture_raw', 0),
                    'fan': 'ON' if data.get('fan', False) else 'OFF',
                    'pump': 'ON' if data.get('pump', False) else 'OFF',
                    'auto_mode': 'ON' if data.get('auto_mode', False) else 'OFF'
                }
                
                writer.writerow(row)
                
        except Exception as e:
            print(f"Error logging to CSV: {e}")
    
    def _cleanup_old_data(self):
        """Remove data files older than retention_days"""
        try:
            cutoff_date = datetime.now() - timedelta(days=self.retention_days)
            cutoff_timestamp = cutoff_date.timestamp()
            
            removed_count = 0
            for filename in os.listdir(self.data_dir):
                filepath = os.path.join(self.data_dir, filename)
                if os.path.isfile(filepath) and filename.endswith('.csv'):
                    # Get file modification time
                    file_mtime = os.path.getmtime(filepath)
                    if file_mtime < cutoff_timestamp:
                        os.remove(filepath)
                        removed_count += 1
            
            if removed_count > 0:
                print(f"Cleaned up {removed_count} old data file(s) (older than {self.retention_days} days)")
        except Exception as e:
            print(f"Error cleaning up old data: {e}")
    
    def get_data_files(self, node_id=None):
        """Get list of data files for a node or all nodes"""
        files = []
        if not os.path.exists(self.data_dir):
            return files
            
        for filename in os.listdir(self.data_dir):
            if filename.endswith('.csv'):
                if node_id is None or filename.startswith(f"{node_id}_"):
                    filepath = os.path.join(self.data_dir, filename)
                    files.append({
                        'filename': filename,
                        'path': filepath,
                        'size': os.path.getsize(filepath),
                        'modified': datetime.fromtimestamp(os.path.getmtime(filepath)).strftime("%Y-%m-%d %H:%M:%S")
                    })
        return sorted(files, key=lambda x: x['filename'], reverse=True)
    
    def delete_node_data(self, node_id, days=None):
        """Delete data files for a node
        Args:
            node_id: Node ID to delete files for
            days: If specified, only delete files older than X days. If None, delete all files.
        Returns:
            List of deleted filenames
        """
        deleted = []
        if not os.path.exists(self.data_dir):
            return deleted
        
        cutoff_timestamp = None
        if days is not None:
            cutoff_date = datetime.now() - timedelta(days=days)
            cutoff_timestamp = cutoff_date.timestamp()
        
        for filename in os.listdir(self.data_dir):
            if filename.endswith('.csv') and filename.startswith(f"{node_id}_"):
                filepath = os.path.join(self.data_dir, filename)
                
                # Check if we should delete this file
                should_delete = False
                if days is None:
                    # Delete all files
                    should_delete = True
                else:
                    # Delete only files older than cutoff
                    file_mtime = os.path.getmtime(filepath)
                    if file_mtime < cutoff_timestamp:
                        should_delete = True
                
                if should_delete:
                    try:
                        os.remove(filepath)
                        deleted.append(filename)
                        print(f"Deleted data file: {filename}")
                    except Exception as e:
                        print(f"Error deleting {filename}: {e}")
        
        return deleted

