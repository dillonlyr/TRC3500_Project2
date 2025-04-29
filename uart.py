import serial
import numpy as np
import matplotlib.pyplot as plt

# === CONFIGURATIONS ===
PORT = '/dev/tty.usbmodem11103'   # <-- Change to your real port
BAUDRATE = 115200
TOTAL_SAMPLES = 1000
SAMPLING_RATE = 10000  # Hz
CENTER_VOLTAGE = 1.65  # Center the plot at 1.65V
VOLTAGE_RANGE = 1.65   # +/- around 1.65V, so Y-axis from 0V to 3.3V

# === SERIAL CONNECTION ===
ser = serial.Serial(PORT, BAUDRATE)
print(f"Connected to {PORT}")

# === REAL-TIME PLOTTING SETUP ===
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots(figsize=(14, 6))
line, = ax.plot([], [], lw=2)
line.set_antialiased(True)

ax.set_xlabel('Time (seconds)', fontsize=14)
ax.set_ylabel('Voltage (V)', fontsize=14)
ax.set_title('Voltage vs Time (Live)', fontsize=16)
ax.set_ylim(CENTER_VOLTAGE - VOLTAGE_RANGE, CENTER_VOLTAGE + VOLTAGE_RANGE)
ax.grid(True)

# Remove white space
fig.tight_layout(pad=0.3)  # <-- Corrected: call tight_layout on fig, not ax

# === LOOP FOREVER ===
while True:
    voltages = []
    
    print("\nWaiting for data...")

    while len(voltages) < TOTAL_SAMPLES:
        try:
            serial_line = ser.readline().decode('utf-8').strip()
            voltage = float(serial_line)
            voltages.append(voltage)
        except Exception as e:
            print(f"Error reading line: {serial_line} -> {e}")

    voltages = np.array(voltages)
    time_values = np.arange(0, TOTAL_SAMPLES) / SAMPLING_RATE

    line.set_data(time_values, voltages)
    ax.set_xlim(0, time_values[-1])
    fig.canvas.draw()
    fig.canvas.flush_events()

    # === PRINT MAX AND MIN VOLTAGE ===
    max_voltage = np.max(voltages)
    min_voltage = np.min(voltages)
    print(f"Max Voltage: {max_voltage:.3f} V")
    print(f"Min Voltage: {min_voltage:.3f} V")
    print("----------------------------------")

    print("Frame complete — waiting for next frame...")
