#!/usr/bin/env python3
"""
Bridge Monitoring Mock Data Generator
Generates realistic sensor data for testing Grafana dashboard
Preserves original serial handler for team integration
"""
import sys
import json
import time
import requests
import math
import random
from datetime import datetime
from threading import Thread

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
        ultrasonic_distance = data.get('ultrasonic_distance', 0)
        accel_x = data.get('accel_x', 0)
        accel_y = data.get('accel_y', 0)
        accel_z = data.get('accel_z', 0)
        
        # Calculate vibration RMS
        vibration_rms = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
        
        # Sensor data packet
        sensor_tags = {"sensor_node": data.get('node_id', 'bridge_01')}
        sensor_fields = {
            "ultrasonic_distance": ultrasonic_distance,
            "accel_x": accel_x,
            "accel_y": accel_y,
            "accel_z": accel_z,
            "vibration_rms": vibration_rms
        }
        
        # Add optional fields if present
        if 'temperature' in data:
            sensor_fields['temperature'] = data['temperature']
        if 'humidity' in data:
            sensor_fields['humidity'] = data['humidity']
        if 'battery_voltage' in data:
            sensor_fields['battery_voltage'] = data['battery_voltage']
        
        packets.append(create_line_protocol("sensor_data", sensor_tags, sensor_fields, timestamp_ns))
        
        # Calculate safety metrics
        safety_score = calculate_safety_score(ultrasonic_distance, vibration_rms)
        boom_gate_status = determine_boom_gate_status(ultrasonic_distance, vibration_rms, safety_score)
        
        # Bridge status packet
        status_tags = {"bridge_id": data.get('bridge_id', 'main_bridge')}
        status_fields = {
            "safety_score": safety_score,
            "boom_gate_status": boom_gate_status,
            "clearance_status": 1 if ultrasonic_distance <= WARNING_CLEARANCE else 0,
            "vibration_status": 1 if vibration_rms >= WARNING_VIBRATION else 0
        }
        packets.append(create_line_protocol("bridge_status", status_tags, status_fields, timestamp_ns))
        
        # Generate alerts if necessary
        if boom_gate_status == 2:  # CLOSED
            alert_tags = {"alert_type": "CRITICAL", "bridge_id": data.get('bridge_id', 'main_bridge')}
            alert_fields = {"message": f"\"Bridge closed: clearance={ultrasonic_distance}cm, vibration={vibration_rms:.3f}g\""}
            packets.append(create_line_protocol("alerts", alert_tags, alert_fields, timestamp_ns))
        elif boom_gate_status == 1:  # WARN
            alert_tags = {"alert_type": "WARNING", "bridge_id": data.get('bridge_id', 'main_bridge')}
            alert_fields = {"message": f"\"Bridge warning: clearance={ultrasonic_distance}cm, vibration={vibration_rms:.3f}g\""}
            packets.append(create_line_protocol("alerts", alert_tags, alert_fields, timestamp_ns))
        
        return "\n".join(packets)
        
    except Exception as e:
        print(f"Error processing sensor data: {e}")
        return None

class MockDataGenerator:
    """Generate realistic bridge sensor data for testing"""
    
    def __init__(self):
        self.time_offset = 0
        self.base_clearance = 250  # Normal water clearance
        self.base_accel = 0.1  # Base vibration level
        self.scenarios = [
            {"name": "Normal Operation", "duration": 60, "clearance_range": (200, 300), "vibration_mult": 1.0},
            {"name": "High Water Warning", "duration": 45, "clearance_range": (70, 120), "vibration_mult": 1.2},
            {"name": "Critical Water Level", "duration": 30, "clearance_range": (20, 60), "vibration_mult": 2.0},
            {"name": "Heavy Traffic Vibration", "duration": 40, "clearance_range": (180, 250), "vibration_mult": 3.0},
            {"name": "Storm Conditions", "duration": 35, "clearance_range": (30, 80), "vibration_mult": 4.0},
            {"name": "Recovery to Normal", "duration": 50, "clearance_range": (150, 280), "vibration_mult": 0.8},
        ]
        self.current_scenario = 0
        self.scenario_start_time = time.time()
    
    def get_current_scenario(self):
        """Get current test scenario"""
        elapsed = time.time() - self.scenario_start_time
        scenario = self.scenarios[self.current_scenario]
        
        if elapsed > scenario["duration"]:
            self.current_scenario = (self.current_scenario + 1) % len(self.scenarios)
            self.scenario_start_time = time.time()
            scenario = self.scenarios[self.current_scenario]
            print(f"\n--- Switching to scenario: {scenario['name']} ---")
        
        return scenario
    
    def generate_sensor_data(self):
        """Generate realistic sensor data based on current scenario"""
        scenario = self.get_current_scenario()
        
        # Generate clearance with some noise and trends
        clearance_min, clearance_max = scenario["clearance_range"]
        trend = math.sin(time.time() / 30) * 20  # Slow trend
        noise = random.gauss(0, 10)  # Random noise
        clearance = random.uniform(clearance_min, clearance_max) + trend + noise
        clearance = max(10, min(400, clearance))  # Clamp to realistic range
        
        # Generate accelerometer data with scenario-based vibration
        vibration_base = self.base_accel * scenario["vibration_mult"]
        
        # Add some realistic vibration patterns
        traffic_vibration = abs(math.sin(time.time() * 2)) * vibration_base * 0.5
        random_vibration = random.gauss(0, vibration_base * 0.3)
        
        accel_x = vibration_base + traffic_vibration + random_vibration
        accel_y = vibration_base * 0.8 + random.gauss(0, vibration_base * 0.2)
        accel_z = vibration_base * 0.6 + random.gauss(0, vibration_base * 0.15)
        
        # Add occasional spikes for realism
        if random.random() < 0.05:  # 5% chance of spike
            spike_mult = random.uniform(2, 5)
            accel_x *= spike_mult
            accel_y *= spike_mult * 0.8
            accel_z *= spike_mult * 0.6
        
        # Environmental data
        temperature = 22 + random.gauss(0, 3)  # Ambient temperature
        humidity = 60 + random.gauss(0, 10)  # Humidity
        battery_voltage = 3.3 + random.gauss(0, 0.1)  # Battery level
        
        return {
            "node_id": "bridge_01",
            "bridge_id": "main_bridge",
            "ultrasonic_distance": round(clearance, 1),
            "accel_x": round(accel_x, 3),
            "accel_y": round(accel_y, 3),
            "accel_z": round(accel_z, 3),
            "temperature": round(temperature, 1),
            "humidity": round(humidity, 1),
            "battery_voltage": round(battery_voltage, 2),
            "timestamp": datetime.now().isoformat()
        }

def run_mock_data_generator():
    """Main function to run mock data generation"""
    print("Starting Bridge Monitoring Mock Data Generator")
    print(f"Sending data to InfluxDB: {INFLUX_URL}")
    print("Mock scenarios will cycle automatically")
    print("Press Ctrl+C to stop\n")
    
    generator = MockDataGenerator()
    packet_count = 0
    error_count = 0
    
    try:
        while True:
            # Generate mock sensor data
            sensor_data = generator.generate_sensor_data()
            packet_count += 1
            
            # Process and send to InfluxDB
            payload = process_sensor_data(sensor_data)
            if payload and send_to_influxdb(payload):
                if packet_count % 10 == 0:  # Print status every 10 packets
                    vibration_rms = math.sqrt(
                        sensor_data['accel_x']**2 + 
                        sensor_data['accel_y']**2 + 
                        sensor_data['accel_z']**2
                    )
                    safety_score = calculate_safety_score(sensor_data['ultrasonic_distance'], vibration_rms)
                    boom_status = ["OPEN", "WARN", "CLOSED"][determine_boom_gate_status(
                        sensor_data['ultrasonic_distance'], vibration_rms, safety_score
                    )]
                    
                    current_scenario = generator.get_current_scenario()
                    print(f"Packet #{packet_count} | Scenario: {current_scenario['name']}")
                    print(f"  Clearance: {sensor_data['ultrasonic_distance']}cm | Vibration: {vibration_rms:.3f}g")
                    print(f"  Safety Score: {safety_score:.1f}% | Boom Gate: {boom_status}")
                    print(f"  Temp: {sensor_data['temperature']}°C | Battery: {sensor_data['battery_voltage']}V")
            else:
                error_count += 1
                print(f"Failed to send packet #{packet_count}")
            
            # Wait before next reading (simulate sensor frequency)
            time.sleep(2)  # Send data every 2 seconds
            
    except KeyboardInterrupt:
        print(f"\nStopping Mock Data Generator")
        print(f"Total packets sent: {packet_count}")
        print(f"Total errors: {error_count}")
        
    except Exception as e:
        print(f"Unexpected error: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--help":
        print("Bridge Monitoring Mock Data Generator")
        print("Usage: python3 bridge_mock_generator.py")
        print("\nThis generates realistic bridge sensor data for testing the Grafana dashboard.")
        print("Data includes various scenarios like normal operation, high water, and storm conditions.")
        print("The scenarios cycle automatically to test all dashboard features.")
        sys.exit(0)
    
    run_mock_data_generator()