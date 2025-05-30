import serial
import time
import math
import random
import requests
from datetime import datetime

# Serial config
SERIAL_PORT = '/dev/cu.usbserial-140'
BAUD_RATE = 115200

# InfluxDB config
INFLUX_URL = "http://localhost:8086/api/v2/write?org=bridge_org&bucket=bridge_monitoring&precision=ns"
TOKEN = "bridge_monitor_token"
HEADERS = {"Authorization": f"Token {TOKEN}"}

# Distance thresholds (cm): >15 open, 10-15 warn, <10 close
CRITICAL_CLEARANCE = 10  # cm - below this = CLOSED
WARNING_CLEARANCE = 15   # cm - below this = WARN

# Vibration thresholds (g): <1.00 open, 1.00-1.50 warn, >1.50 close
WARNING_VIBRATION = 1.0   # g - above this = WARN
CRITICAL_VIBRATION = 1.5  # g - above this = CLOSED

PREFIX = "[ESP-DEVKIT-Result] ASCII interpretation: '"

def send_to_influxdb(payload):
    try:
        response = requests.post(INFLUX_URL, headers=HEADERS, data=payload.encode())
        if response.status_code != 204:
            print(f"[ERROR] InfluxDB write failed: {response.status_code} {response.text}")
            return False
        return True
    except Exception as e:
        print(f"[ERROR] Failed to send data: {e}")
        return False

def parse_esp_devkit_line(line):
    if line.startswith(PREFIX):
        try:
            content = line[len(PREFIX):-1]  # Remove prefix and trailing `'`
            vibration_str, distance_timestamp = content.split(',')
            distance_str, _ = distance_timestamp.split('@')

            vibration = float(vibration_str.strip())
            distance = float(distance_str.strip())

            # Current time in nanoseconds
            timestamp_ns = int(time.time() * 1e9)

            return distance, vibration, timestamp_ns
        except Exception as e:
            print(f"[WARN] Failed to parse line: {line.strip()} | Error: {e}")
    return None, None, None

def acc_data(vibration_rms):
    
    base_factor = vibration_rms / math.sqrt(3)  # Base distribution
    

    accel_x = base_factor * random.uniform(1.2, 1.8)  # X-axis dominant
    accel_y = base_factor * random.uniform(0.6, 1.0)  # Y-axis moderate
    accel_z = base_factor * random.uniform(0.4, 0.8)  # Z-axis minimal
    

    calculated_rms = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    if calculated_rms > 0:
        scale_factor = vibration_rms / calculated_rms
        accel_x *= scale_factor
        accel_y *= scale_factor
        accel_z *= scale_factor
    
    return round(accel_x, 3), round(accel_y, 3), round(accel_z, 3)

def env_data():

    temperature = round(22 + random.gauss(0, 3), 1) 
    humidity = round(60 + random.gauss(0, 10), 1)    
    battery_voltage = round(3.3 + random.gauss(0, 0.1), 2)  
    
    temperature = max(10, min(35, temperature))
    humidity = max(30, min(90, humidity))
    battery_voltage = max(3.0, min(3.6, battery_voltage))
    
    return temperature, humidity, battery_voltage

def create_line_protocol(distance, vibration_rms, timestamp_ns):

    accel_x, accel_y, accel_z = acc_data(vibration_rms)
    

    temperature, humidity, battery_voltage = env_data()
    

    safety_score = calculate_safety_score(distance, vibration_rms)
    boom_status = determine_boom_gate_status(distance, vibration_rms)
    
    clearance_status = int(distance < WARNING_CLEARANCE)  # 1 if distance < 15cm
    vibration_status = int(vibration_rms >= WARNING_VIBRATION)  # 1 if vibration >= 1.0g


    sensor_fields = [
        f"ultrasonic_distance={distance}",
        f"accel_x={accel_x}",
        f"accel_y={accel_y}", 
        f"accel_z={accel_z}",
        f"vibration_rms={vibration_rms}",
        f"temperature={temperature}",
        f"humidity={humidity}",
        f"battery_voltage={battery_voltage}"
    ]
    sensor_line = f"sensor_data,sensor_node=bridge_01 {','.join(sensor_fields)} {timestamp_ns}"

    # Bridge status
    status_fields = [
        f"safety_score={safety_score}",
        f"boom_gate_status={boom_status}",
        f"clearance_status={clearance_status}",
        f"vibration_status={vibration_status}"
    ]
    status_line = f"bridge_status,bridge_id=main_bridge {','.join(status_fields)} {timestamp_ns}"

    # Alert logic based on boom gate status
    alert_line = ""
    if boom_status == 2:  # CLOSED
        message = f"Bridge closed: clearance={distance:.1f}cm, vibration={vibration_rms:.3f}g"
        alert_line = f"alerts,alert_type=CRITICAL,bridge_id=main_bridge message=\"{message}\" {timestamp_ns}"
    elif boom_status == 1:  # WARN
        message = f"Bridge warning: clearance={distance:.1f}cm, vibration={vibration_rms:.3f}g"
        alert_line = f"alerts,alert_type=WARNING,bridge_id=main_bridge message=\"{message}\" {timestamp_ns}"

    # Combine all lines
    lines = [sensor_line, status_line]
    if alert_line:
        lines.append(alert_line)
    
    return "\n".join(lines)

def calculate_safety_score(clearance, vibration_rms):

    clearance_score = 0
    vibration_score = 0
    
    # Clearance scoring (0-50 points)
    if clearance > WARNING_CLEARANCE:  # >15cm
        clearance_score = 50  # Full points for safe clearance
    elif clearance >= CRITICAL_CLEARANCE:  # 10-15cm (warning zone)
        # Linear scale from 25 to 50 points in warning zone
        clearance_score = 25 + (clearance - CRITICAL_CLEARANCE) * 25 / (WARNING_CLEARANCE - CRITICAL_CLEARANCE)
    else:  # <10cm (critical zone)
        # Scale from 0 to 25 points in critical zone
        clearance_score = max(0, clearance * 25 / CRITICAL_CLEARANCE)
    
    # Vibration scoring (0-50 points)
    if vibration_rms < WARNING_VIBRATION:  # <1.0g
        vibration_score = 50  # Full points for low vibration
    elif vibration_rms <= CRITICAL_VIBRATION:  # 1.0-1.5g (warning zone)
        # Linear scale from 50 to 25 points in warning zone
        vibration_score = 50 - ((vibration_rms - WARNING_VIBRATION) * 25 / (CRITICAL_VIBRATION - WARNING_VIBRATION))
    else:  # >1.5g (critical zone)
        # Rapidly decreasing points above critical threshold
        vibration_score = max(0, 25 - (vibration_rms - CRITICAL_VIBRATION) * 10)
    
    total_score = clearance_score + vibration_score
    return round(min(100, max(0, total_score)), 1)

def determine_boom_gate_status(clearance, vibration_rms):

    # Check for CRITICAL conditions first (CLOSED)
    if clearance < CRITICAL_CLEARANCE or vibration_rms > CRITICAL_VIBRATION:
        return 2  # CLOSED
    
    # Check for WARNING conditions (WARN)  
    elif clearance < WARNING_CLEARANCE or vibration_rms >= WARNING_VIBRATION:
        return 1  # WARN
    
    else:
        return 0  # OPEN

def main():
    print(f"[INFO] Enhanced Serial Handler - Reading from: {SERIAL_PORT}")
    print(f"[INFO] Updated Safety Thresholds:")
    print(f"  Distance: >15cm=OPEN, 10-15cm=WARN, <10cm=CLOSED")
    print(f"  Vibration: <1.0g=OPEN, 1.0-1.5g=WARN, >1.5g=CLOSED")
    print(f"[INFO] Sending enhanced payload to InfluxDB")
    
    packet_count = 0
    
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            while True:
                try:
                    line = ser.readline().decode('utf-8').strip()
                    if not line:
                        continue
                        
                    distance, vibration_rms, timestamp_ns = parse_esp_devkit_line(line)
                    if distance is not None and vibration_rms is not None:
                        packet_count += 1
                        
                        # Create enhanced payload
                        payload = create_line_protocol(distance, vibration_rms, timestamp_ns)
                        
                        # Calculate display values
                        safety_score = calculate_safety_score(distance, vibration_rms)
                        boom_status = determine_boom_gate_status(distance, vibration_rms)
                        boom_status_text = ["OPEN", "WARN", "CLOSED"][boom_status]
                        
                        print(f"[INFO] Packet #{packet_count}")
                        print(f"  Real Data: Clearance={distance}cm | Vibration={vibration_rms}g")
                        print(f"  Status: Safety={safety_score}% | Boom Gate={boom_status_text}")
                        
                        # Send to InfluxDB
                        if send_to_influxdb(payload):
                            print(f"  ✓ Sent enhanced payload to InfluxDB")
                        else:
                            print(f"  ✗ Failed to send payload")
                        print()
                        
                except serial.SerialException as e:
                    print(f"[ERROR] Serial communication error: {e}")
                    break
                except Exception as e:
                    print(f"[ERROR] Unexpected error processing data: {e}")
                    continue
                    
    except serial.SerialException as e:
        print(f"[ERROR] Failed to open serial port {SERIAL_PORT}: {e}")
        print("[INFO] Make sure the ESP32 is connected and the port is correct")
    except KeyboardInterrupt:
        print(f"\n[INFO] Stopped by user after processing {packet_count} packets")
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")

if __name__ == "__main__":
    main()
