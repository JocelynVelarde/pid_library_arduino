import streamlit as st
import serial
import json
import pandas as pd
import time

SERIAL_PORT = 'COM10'
BAUD_RATE = 115200
MAX_HISTORY = 100    

st.set_page_config(page_title="Motor PID Dashboard", layout="wide")
st.title("⚙️ ESP32 Motor PID Controller")

metrics_col1, metrics_col2, metrics_col3 = st.columns(3)
target_metric = metrics_col1.empty()
real_metric = metrics_col2.empty()
status_metric = metrics_col3.empty()

st.divider()

chart_placeholder = st.empty()

@st.cache_resource
def init_serial():
    try:
        return serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        st.error(f"Could not open serial port {SERIAL_PORT}: {e}")
        return None

ser = init_serial()
data_history = []

if ser is not None:
    st.success(f"Connected to {SERIAL_PORT}. Waiting for data...")
    last_ui_update = time.time()
    
    while True:
        try:
            lines_processed = 0
            
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                if line.startswith("{") and line.endswith("}"):
                    data = json.loads(line)
                    data_history.append(data)
                    lines_processed += 1
                    
            if len(data_history) > MAX_HISTORY:
                data_history = data_history[-MAX_HISTORY:]
            current_time = time.time()
            if lines_processed > 0 and (current_time - last_ui_update) > 0.1:
                latest_data = data_history[-1] 
                
                target_metric.metric("Target Speed", f"{latest_data['target']:.1f}%")
                real_metric.metric("Actual Speed", f"{latest_data['real']:.1f}%")

                if latest_data['stop'] == 1:
                    status_metric.error("🚨 MOTOR STOPPED")
                else:
                    status_metric.success("✅ RUNNING")

                df = pd.DataFrame(data_history)
                chart_placeholder.line_chart(df[['target', 'real']])
                
                last_ui_update = current_time 
                
            time.sleep(0.01)
            
        except json.JSONDecodeError:
            pass 
        except Exception as e:
            st.error(f"Error reading data: {e}")
            time.sleep(1)