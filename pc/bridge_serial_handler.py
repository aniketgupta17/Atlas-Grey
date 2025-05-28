#!/usr/bin/env python3
"""
Bridge Monitoring Serial to InfluxDB Handler
Receives sensor data from ESP32 and stores in InfluxDB
Original version for team member to integrate with HTTP endpoints
"""
import sys
import json
import time
import serial
import requests
import math
from datetime import datetime

#
# ESP32 JSON Data Format Guide (If sending JSON over serial in the future):
# -----------------------------------------------------------------------
# If you choose to send data from the ESP32 as JSON strings (one per line),
# the following structure and fields are recommended for compatibility with
# the process_sensor_data function.
#
# Each JSON string should be a single line terminated by a newline character ('\n').
# Example:
# {"node_id": "esp32_node_01", "bridge_id": "main_bridge", "ultrasonic_distance": 35.2, "vibration_rms": 0.98, "accel_x": 0.1, "accel_y": 0.05, "accel_z": 0.9, "temperature": 25.5, "humidity": 60.1, "battery_voltage": 3.25, "device_timestamp": "10:35:15"}
#
# Fields:
#   - "node_id": (String, Optional, default: "bridge_01" or derived) Identifier for the sensor node.
#   - "bridge_id": (String, Optional, default: "main_bridge" or derived) Identifier for the bridge.
#   - "ultrasonic_distance": (Float, Required) Distance measured by ultrasonic sensor, IN CENTIMETERS (cm).
#   - "vibration_rms": (Float, Optional) Pre-calculated Root Mean Square of vibration, IN 'g'.
#                      If provided, this value will be used directly. This corresponds to 'vibration_rms_override'
#                      when parsing custom formats.
#   - "accel_x": (Float, Optional) Acceleration on X-axis, IN 'g'.
#                Required if "vibration_rms" is not provided, for on-script calculation.
#   - "accel_y": (Float, Optional) Acceleration on Y-axis, IN 'g'.
#                Required if "vibration_rms" is not provided.
#   - "accel_z": (Float, Optional) Acceleration on Z-axis, IN 'g'.
#                Required if "vibration_rms" is not provided.
#   - "temperature": (Float, Optional) Ambient temperature, IN CELSIUS (°C).
#   - "humidity": (Float, Optional) Relative humidity, IN PERCENT (%).
#   - "battery_voltage": (Float, Optional) Sensor node battery voltage, IN VOLTS (V).
#   - "device_timestamp": (String, Optional) Timestamp from the ESP32 (e.g., "HH:MM:SS" or ISO8601).
#                         This will be stored as a string field in InfluxDB.
#
# Notes:
#   - If "vibration_rms" is sent (as 'vibration_rms_override' from custom format or 'vibration_rms' in JSON),
#     "accel_x", "accel_y", "accel_z" are optional but can still be sent for raw data logging if included in the JSON.
#   - If "vibration_rms" is NOT sent via JSON (and no 'vibration_rms_override' from custom format),
#     "accel_x", "accel_y", "accel_z" ARE REQUIRED in the JSON for the script to calculate RMS vibration.
#   - Ensure all floating point numbers are standard decimal representations (e.g., 123.45).
#

# InfluxDB configuration
INFLUX_URL = "http://localhost:8086/api/v2/write?org=bridge_org&bucket=bridge_monitoring&precision=ns"
TOKEN = "bridge_monitor_token"
HEADERS = {"Authorization": f"Token {TOKEN}"}

# Safety thresholds
CRITICAL_CLEARANCE = 50  # cm
WARNING_CLEARANCE = 100  # cm
CRITICAL_VIBRATION = 1.5  # g
WARNING_VIBRATION = 1.0  # g

def create_line_protocol(measurement, tags, fields, timestamp_ns):
    """Create InfluxDB line protocol format"""
    tag_str = ",".join(f"{k}={v}" for k, v in tags.items())
    field_str = ",".join(f"{k}={v}" for k, v in fields.items())
    return f"{measurement},{tag_str} {field_str} {timestamp_ns}"

def calculate_safety_score(clearance, vibration_rms):
    """Calculate overall bridge safety score (0-100)"""
    clearance_score = 0
    vibration_score = 0
    
    # Clearance scoring (0-50 points)
    if clearance >= WARNING_CLEARANCE:
        clearance_score = 50
    elif clearance >= CRITICAL_CLEARANCE:
        clearance_score = 25 + (clearance - CRITICAL_CLEARANCE) * 25 / (WARNING_CLEARANCE - CRITICAL_CLEARANCE)
    else:
        clearance_score = max(0, clearance * 25 / CRITICAL_CLEARANCE)
    
    # Vibration scoring (0-50 points)
    if vibration_rms <= WARNING_VIBRATION:
        vibration_score = 50 - (vibration_rms * 25 / WARNING_VIBRATION)
    elif vibration_rms <= CRITICAL_VIBRATION:
        vibration_score = 25 - ((vibration_rms - WARNING_VIBRATION) * 25 / (CRITICAL_VIBRATION - WARNING_VIBRATION))
    else:
        vibration_score = max(0, 25 - (vibration_rms - CRITICAL_VIBRATION) * 5)
    
    return min(100, max(0, clearance_score + vibration_score))

def determine_boom_gate_status(clearance, vibration_rms, safety_score):
    """Determine boom gate status based on sensor readings"""
    # 0 = OPEN, 1 = WARN, 2 = CLOSED
    if clearance <= CRITICAL_CLEARANCE or vibration_rms >= CRITICAL_VIBRATION:
        return 2  # CLOSED
    elif clearance <= WARNING_CLEARANCE or vibration_rms >= WARNING_VIBRATION or safety_score < 70:
        return 1  # WARN
    else:
        return 0  # OPEN

def send_to_influxdb(payload):
    """Send data to InfluxDB with error handling"""
    try:
        response = requests.post(INFLUX_URL, headers=HEADERS, data=payload.encode())
        if response.status_code != 204:
            print(f"Error sending data: {response.status_code} {response.text}")
            return False
        return True
    except Exception as e:
        print(f"Failed to send data: {e}")
        return False

def process_sensor_data(data):
    """Process incoming sensor data and create InfluxDB records"""
    try:
        timestamp_ns = time.time_ns()
        packets = []
        
        # Extract sensor readings
        ultrasonic_distance = data.get('ultrasonic_distance', 0) # Expected in cm

        accel_x_val = data.get('accel_x') # Raw accel data, might be None
        accel_y_val = data.get('accel_y')
        accel_z_val = data.get('accel_z')

        if 'vibration_rms_override' in data:
            vibration_rms = data['vibration_rms_override']
        elif accel_x_val is not None and accel_y_val is not None and accel_z_val is not None:
            vibration_rms = math.sqrt(accel_x_val**2 + accel_y_val**2 + accel_z_val**2)
        else:
            vibration_rms = data.get('vibration_rms', 0) # Default if no override or accel data

        # Sensor data packet
        sensor_tags = {"sensor_node": data.get('node_id', 'bridge_01')}
        sensor_fields = {
            "ultrasonic_distance": ultrasonic_distance, # Already in cm
            "vibration_rms": vibration_rms
        }
        
        if accel_x_val is not None:
            sensor_fields["accel_x"] = accel_x_val
        if accel_y_val is not None:
            sensor_fields["accel_y"] = accel_y_val
        if accel_z_val is not None:
            sensor_fields["accel_z"] = accel_z_val
        
        if 'temperature' in data:
            sensor_fields['temperature'] = data['temperature']
        if 'humidity' in data:
            sensor_fields['humidity'] = data['humidity']
        if 'battery_voltage' in data:
            sensor_fields['battery_voltage'] = data['battery_voltage']
        if 'device_timestamp' in data:
            # Properly quote string field for InfluxDB line protocol
            dt_val = str(data['device_timestamp']).replace('"', '\\"') # Escape existing double quotes
            sensor_fields['device_timestamp'] = f'""{dt_val}""' # Enclose in double quotes

        packets.append(create_line_protocol("sensor_data", sensor_tags, sensor_fields, timestamp_ns))
        
        safety_score = calculate_safety_score(ultrasonic_distance, vibration_rms)
        boom_gate_status = determine_boom_gate_status(ultrasonic_distance, vibration_rms, safety_score)
        
        status_tags = {"bridge_id": data.get('bridge_id', 'main_bridge')}
        status_fields = {
            "safety_score": safety_score,
            "boom_gate_status": boom_gate_status,
            "clearance_status": 1 if ultrasonic_distance <= WARNING_CLEARANCE else 0,
            "vibration_status": 1 if vibration_rms >= WARNING_VIBRATION else 0
        }
        packets.append(create_line_protocol("bridge_status", status_tags, status_fields, timestamp_ns))
        
        if boom_gate_status == 2:  # CLOSED
            alert_tags = {"alert_type": "CRITICAL", "bridge_id": data.get('bridge_id', 'main_bridge')}
            # Ensure message string is correctly quoted for InfluxDB, escaping internal quotes
            message = f"Bridge closed: clearance={ultrasonic_distance}cm, vibration={vibration_rms:.3f}g"
            escaped_message = message.replace('"', '\\"')
            alert_fields = {"message": f'""{escaped_message}""'}
            packets.append(create_line_protocol("alerts", alert_tags, alert_fields, timestamp_ns))
        elif boom_gate_status == 1:  # WARN
            alert_tags = {"alert_type": "WARNING", "bridge_id": data.get('bridge_id', 'main_bridge')}
            message = f"Bridge warning: clearance={ultrasonic_distance}cm, vibration={vibration_rms:.3f}g"
            escaped_message = message.replace('"', '\\"')
            alert_fields = {"message": f'""{escaped_message}""'}
            packets.append(create_line_protocol("alerts", alert_tags, alert_fields, timestamp_ns))
        
        return "\n".join(packets)
        
    except Exception as e:
        print(f"Error processing sensor data: {e} (data: {data})")
        return None

def main_serial(port="/dev/ttyUSB0", baud_rate=115200):
    """Main function to handle serial communication"""
    print(f"Starting Bridge Monitoring System (Serial Mode)")
    print(f"Connecting to {port} at {baud_rate} baud")
    print(f"Sending data to InfluxDB: {INFLUX_URL}")
    print("Parsing custom format: vibration_rms,ultrasonic_mm@timestamp")
    print("Press Ctrl+C to stop\n")
    
    ser_conn = None
    try:
        ser_conn = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)
        ser_conn.flushInput()
        
        packet_count = 0
        error_count = 0
        
        while True:
            line = ""
            try:
                if ser_conn.in_waiting > 0:
                    line = ser_conn.readline().decode('utf-8').strip()
                
                if not line:
                    time.sleep(0.05)
                    continue
                
                parts = line.split('@')
                if len(parts) != 2:
                    print(f"Invalid format (missing '@' or too many parts): {line}")
                    error_count += 1
                    continue
                
                data_part, device_time_str = parts[0], parts[1]
                
                sensor_values = data_part.split(',')
                if len(sensor_values) != 2:
                    print(f"Invalid data format (expected 2 sensor values, got {len(sensor_values)}): {data_part}")
                    error_count += 1
                    continue
                    
                vibration_rms_str, ultrasonic_mm_str = sensor_values[0], sensor_values[1]
                
                try:
                    vibration_rms_val = float(vibration_rms_str)
                    ultrasonic_mm_val = float(ultrasonic_mm_str)
                except ValueError:
                    print(f"Invalid sensor numeric values: vibration='{vibration_rms_str}', ultrasonic='{ultrasonic_mm_str}' in line: {line}")
                    error_count += 1
                    continue

                ultrasonic_cm_val = ultrasonic_mm_val / 10.0
                packet_count += 1
                
                node_id_suffix = port.split('/')[-1]
                sensor_data_dict = {
                    'node_id': f"esp32_{node_id_suffix}", 
                    'bridge_id': 'main_bridge_serial',
                    'ultrasonic_distance': ultrasonic_cm_val,
                    'vibration_rms_override': vibration_rms_val,
                    'device_timestamp': device_time_str
                }
                
                payload = process_sensor_data(sensor_data_dict)
                if payload and send_to_influxdb(payload):
                    if packet_count % 10 == 0:
                        print(f"Packet #{packet_count}: Clearance={sensor_data_dict.get('ultrasonic_distance', 'N/A'):.1f}cm, "
                              f"Vibration={sensor_data_dict.get('vibration_rms_override', 'N/A'):.3f}g, "
                              f"ESP_Time={sensor_data_dict.get('device_timestamp', 'N/A')}")
                else:
                    error_count += 1
                    if payload is None:
                        print(f"Failed to process packet #{packet_count}. Line: '{line}'")
                    else:
                        print(f"Failed to send packet #{packet_count} to InfluxDB. Line: '{line}'")
                        
            except serial.SerialException as e:
                print(f"Serial error: {e}. Attempting to reconnect...")
                if ser_conn and ser_conn.is_open:
                    ser_conn.close()
                time.sleep(5)
                try:
                    ser_conn = serial.Serial(port, baud_rate, timeout=1)
                    print("Reconnected successfully.")
                    time.sleep(2)
                    ser_conn.flushInput()
                except serial.SerialException as se_recon:
                    print(f"Reconnect failed: {se_recon}")
                    time.sleep(5)
            except UnicodeDecodeError as ude:
                print(f"Unicode decode error processing line: '{line if line else 'empty'}'. Ensure ESP32 sends UTF-8. Error: {ude}")
                error_count += 1
            except Exception as e_loop:
                print(f"Error processing line '{line if line else 'empty'}': {e_loop}")
                error_count +=1
                time.sleep(1)
                
    except KeyboardInterrupt:
        print(f"\nStopping Bridge Monitoring System")
        print(f"Total packets processed: {packet_count}")
        print(f"Total errors: {error_count}")
        
    except Exception as e_main:
        print(f"Unexpected critical error in main_serial: {e_main}")
        
    finally:
        if ser_conn and ser_conn.is_open:
            ser_conn.close()
            print("Serial port closed.")

def main_http_server():
    """
    HTTP endpoint handler for team member to implement
    This is a placeholder showing the structure needed
    """
    from flask import Flask, request, jsonify
    
    app = Flask(__name__)
    
    @app.route('/bridge/sensor-data', methods=['POST'])
    def receive_sensor_data():
        """
        HTTP endpoint to receive sensor data
        Expected JSON format:
        {
            "node_id": "bridge_01",
            "bridge_id": "main_bridge", 
            "ultrasonic_distance": 250.5,
            "accel_x": 0.123,
            "accel_y": 0.045,
            "accel_z": 0.098,
            "temperature": 22.5,
            "humidity": 65.2,
            "battery_voltage": 3.28
        }
        """
        try:
            sensor_data = request.get_json()
            
            if not sensor_data:
                return jsonify({"error": "No JSON data received"}), 400
            
            # Process and send to InfluxDB
            payload = process_sensor_data(sensor_data)
            if payload and send_to_influxdb(payload):
                return jsonify({"status": "success", "message": "Data processed successfully"}), 200
            else:
                return jsonify({"error": "Failed to process data"}), 500
                
        except Exception as e:
            return jsonify({"error": str(e)}), 500
    
    @app.route('/bridge/status', methods=['GET'])
    def get_bridge_status():
        """Get current bridge status (placeholder)"""
        return jsonify({"status": "online", "mode": "http"})
    
    print("Starting Bridge Monitoring HTTP Server")
    print("Endpoints available:")
    print("  POST /bridge/sensor-data - Receive sensor data")
    print("  GET  /bridge/status - Get bridge status")
    print("Server running on http://localhost:5000")
    
    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Bridge Monitoring Serial Handler")
        print("Usage:")
        print("  Serial mode: python3 bridge_serial_handler.py serial <serial_port> [baud_rate]")
        print("  HTTP mode:   python3 bridge_serial_handler.py http")
        print("\nExamples:")
        print("  python3 bridge_serial_handler.py serial /dev/ttyUSB0 115200")
        print("  python3 bridge_serial_handler.py http")
        sys.exit(1)
    
    mode = sys.argv[1].lower()
    
    if mode == "serial":
        if len(sys.argv) < 3:
            print("Error: Serial port required for serial mode")
            sys.exit(1)
        port = sys.argv[2]
        baud_rate = int(sys.argv[3]) if len(sys.argv) > 3 else 115200
        main_serial(port, baud_rate)
        
    elif mode == "http":
        main_http_server()
        
    else:
        print(f"Error: Unknown mode '{mode}'. Use 'serial' or 'http'")
        sys.exit(1)