import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

#* ==================== CONFIGURATIONS ====================
PORT = '/dev/tty.usbmodem11103'
BAUDRATE = 115200

TOTAL_SAMPLES = 20000
SAMPLING_RATE = 10000  # Hz
CENTER_VOLTAGE = 1650
VOLTAGE_RANGE = 1650
#* ==================== =============== ====================

# Toggle behavior
signal_focus_mode = False  # True = zoom into signal; False = show full 2s window

# === SERIAL CONNECTION ===
ser = serial.Serial(PORT, BAUDRATE)
print(f"Connected to {PORT}")

# === PLOTTING SETUP ===
plt.ion()
fig, (ax_time, ax_freq) = plt.subplots(2, 1, figsize=(12, 8))
line_time, = ax_time.plot([], [], linewidth=2)
line_freq, = ax_freq.plot([], [], linewidth=2)

# Setup Time Domain Plot
ax_time.set_xlabel('Time (seconds)', fontsize=14)
ax_time.set_ylabel('Voltage (V)', fontsize=14)
ax_time.set_title('Voltage vs Time', fontsize=16)
ax_time.set_ylim(CENTER_VOLTAGE - VOLTAGE_RANGE, CENTER_VOLTAGE + VOLTAGE_RANGE)
ax_time.grid(True)

# Setup Frequency Domain Plot
ax_freq.set_xlabel('Frequency (Hz)', fontsize=14)
ax_freq.set_ylabel('Magnitude', fontsize=14)
ax_freq.set_title('FFT Magnitude Spectrum', fontsize=16)
ax_freq.grid(True)

fig.tight_layout(pad=0.4)

def filter_voltage(v_array, noise_threshold=100):

    mean_voltage = np.mean(v_array)

    filtered_v_array = np.copy(v_array)

    for i in range(len(v_array)):
        if abs(v_array[i] - mean_voltage) <= noise_threshold:
            filtered_v_array[i] = mean_voltage  # flatten small oscillations

    return filtered_v_array

def acquire_and_plot_voltages():

    print("Waiting for signal...")

    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line == "START":
            break

    total_bytes = TOTAL_SAMPLES * 2
    data = ser.read(total_bytes)    # Read all ADC samples (2 bytes each)
    adc_array = np.frombuffer(data, dtype=np.uint16)    # Convert bytes into uint16 array

    # Convert ADC to mV
    v_array = (adc_array.astype(np.float32) * 3300) / 4095
    v_array_filtered = filter_voltage(v_array)
    v_array_off_removed = v_array_filtered - np.mean(v_array_filtered)
    window = np.hanning(len(v_array_off_removed))

    #! TEST THIS LINE
    v_array_off_removed = v_array_filtered * v_array_off_removed

    # === Update Time Plot ===
    line_time.set_data(np.arange(len(v_array)) / SAMPLING_RATE, v_array_filtered)
    ax_time.relim()
    ax_time.autoscale_view()

    # === Compute and Update FFT Plot ===
    x = v_array_off_removed * window
    fft_result = np.fft.fft(x)
    
    fft_magnitude = np.abs(fft_result) / len(v_array)
    fft_magnitude_db = 20 * np.log10(fft_magnitude + 1e-12)  # + small value to prevent log(0)
    freqs = np.fft.fftfreq(len(v_array_off_removed), d=1/SAMPLING_RATE)
    half = len(v_array_off_removed) // 2

    line_freq.set_data(freqs[:half], fft_magnitude_db[:half])
    ax_freq.set_xlim(0, freqs[half-1])
    ax_freq.set_xscale('log')
    ax_freq.set_yscale('log')
    ax_freq.relim()
    ax_freq.autoscale_view()
    ax_freq.set_ylabel("Magnitude (dB)")

    # === Refresh Canvas ===
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.05)

    # Data analysis
    threshold = 0.5 * np.max(fft_magnitude[:half])  # 50% of peak magnitude
    indices_above_threshold = np.where(fft_magnitude[:half] >= threshold)[0]

    if len(indices_above_threshold) > 1:
        bandwidth = freqs[indices_above_threshold[-1]] - freqs[indices_above_threshold[0]]
    else:
        bandwidth = 0  # Flat signal (unlikely for a real impact)

    peaks, _ = find_peaks(fft_magnitude[:half], height=np.max(fft_magnitude[:half]) * 0.1)

    # Get the frequencies of those peaks
    dominant_harmonics = freqs[peaks]

    one_sided = fft_result[:half]
    psd = np.abs(one_sided)**2       # energy per bin
    # If you want average (power) rather than cumulative energy:
    avg_energy = np.sum(psd) / len(x)
    # Or for raw energy (which grows with N):
    total_energy = np.sum(psd)         
        
    return v_array_filtered, freqs[np.argmax(fft_magnitude[:half])], avg_energy, total_energy, np.sum(fft_magnitude[:half]), np.sum(freqs[:half] * fft_magnitude[:half]) / np.sum(fft_magnitude[:half]), bandwidth, dominant_harmonics


def rising_count(v_array, center_voltage=1650, threshold=20):

    crossing_count = 0

    lower_bound = center_voltage - threshold
    upper_bound = center_voltage + threshold

    for i in range(1, len(v_array)):
        if v_array[i-1] < lower_bound and v_array[i] > upper_bound:
            crossing_count += 1

    return crossing_count

def falling_count(v_array, center_voltage=1650, threshold=20):

    crossing_count = 0

    lower_bound = center_voltage - threshold
    upper_bound = center_voltage + threshold

    for i in range(1, len(v_array)):
        if v_array[i] < lower_bound and v_array[i - 1] > upper_bound:
            crossing_count += 1

    return crossing_count

# === LOOP ===
while True:

    adc_avg = None
    max_adc = None
    min_adc = None   
    v_avg = None
    max_v = None
    min_v = None
    stm_rises = None
    stm_falls = None
    stm_con = None
    stm_dis = None
    pp = None

    v_arr, peak_freq, avg_energy, total_energy, old_energy, freq_centroid, bandwidth, dom_harmonics = acquire_and_plot_voltages()
    py_mean_v = np.mean(v_arr)
    py_max_v = np.max(v_arr)
    py_min_v = np.min(v_arr)
    
    rises = rising_count(v_arr)
    falls = falling_count(v_arr)

    # Process data
    # if (v_arr)

    # print("\nWaiting for other data...")
    received_adc_avg = False
    received_v_avg = False
    received_max_adc = False
    received_min_adc = False
    received_max_v = False
    received_min_v = False
    received_rise = False
    received_fall = False
    received_con = False
    received_mat = False
    received_pp = False


    while not (received_adc_avg and received_v_avg and 
               received_max_adc and received_min_adc 
               and received_max_v and received_min_v 
               and received_rise and received_fall
               and received_con and received_mat
               and received_pp):
        try:
            serial_line = ser.readline().decode('utf-8').strip()

            # Signal info
            if serial_line.startswith('ADC_AVG:'):
                adc_avg = float(serial_line.split(':')[1])
                received_adc_avg = True
            elif serial_line.startswith('ADC_MAX:'):
                max_adc = int(serial_line.split(':')[1])
                received_max_adc = True
            elif serial_line.startswith('ADC_MIN:'):
                min_adc = int(serial_line.split(':')[1])
                received_min_adc = True
            elif serial_line.startswith('VOLT_AVG:'):
                v_avg = float(serial_line.split(':')[1])
                received_v_avg = True
            elif serial_line.startswith('VOLT_MAX:'):
                max_v = float(serial_line.split(':')[1])
                received_max_v = True
            elif serial_line.startswith('VOLT_MIN:'):
                min_v = float(serial_line.split(':')[1])
                received_min_v = True
            elif serial_line.startswith('RISE_EDGE:'):
                stm_rises = int(serial_line.split(':')[1])
                received_rise = True
            elif serial_line.startswith('FALL_EDGE:'):
                stm_falls = int(serial_line.split(':')[1])
                received_fall = True
            elif serial_line.startswith('PP:'):
                pp = float(serial_line.split(':')[1])
                received_pp = True

            # Classification
            elif serial_line.startswith('CON1'):
                stm_con = "Distance = 10cm; Height = 10cm;"
                received_con = True
            elif serial_line.startswith('CON2'):
                stm_con = "Distance = 10cm; Height = 30cm;"
                received_con = True
            elif serial_line.startswith('CON3'):
                stm_con = "Distance = 30cm; Height = 10cm;"
                received_con = True
            elif serial_line.startswith('CON4'):
                stm_con = "Distance = 30cm; Height = 30cm;"
                received_con = True
            elif serial_line.startswith('E'):
                stm_mat = "ERASER detected!"
                received_mat = True
            elif serial_line.startswith('C'):
                stm_mat = "COIN detected!"
                received_mat = True
            else:
                if serial_line != "":
                    print(f"Ignored unknown data: {serial_line}")

        except Exception as e:
            print(f"Error reading info : {e}")

    # === Print received info ===
    print("==================== STM =====================")
    if adc_avg is not None:
        print(f"STM32 Average ADC value     : {adc_avg:.2f}")
    if max_adc is not None:
        print(f"STM32 Max ADC value         : {max_adc}")
    if min_adc is not None:
        print(f"STM32 Min ADC value         : {min_adc}")
    if v_avg is not None:
        print(f"STM32 Average Voltage       : {v_avg:.2f} mV")
    if max_v is not None:
        print(f"STM32 Max Voltage           : {max_v:.2f} mV")
    if min_v is not None:
        print(f"STM32 Min Voltage           : {min_v:.2f} mV")
    if pp is not None:
        print(f"STM32 PP                    : {pp:.2f} mV")
    if stm_rises is not None:
        print(f"STM32 Rising edges          : {stm_rises}")
    if stm_falls is not None:
        print(f"STM32 Falling edges         : {stm_falls}")
    # if stm_falls is not None:
    #     print(stm_mat)
    # if stm_falls is not None:
    #     print(stm_con)

    # print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n")
    print("=================== PYTHON ===================")
    print(f"PYTHON mean V               : {py_mean_v:.2f} mV")
    print(f"PYTHON max V                : {py_max_v:.2f} mV")
    print(f"PYTHON min V                : {py_min_v:.2f} mV")

    print(f"PYTHON Rising edges         : {rises}")
    print(f"PYTHON Falling edges        : {falls}")

    print(f"PYTHON peak mag. freq       : {peak_freq}")
    print(f"PYTHON bandwidth            : {bandwidth}")
    print(f"PYTHON avg energy           : {avg_energy:.2f}")
    
    print(f"PYTHON total energy (new)   : {total_energy :.2f}")
    print(f"PYTHON total energy (old)   : {old_energy :.2f}")
    print(f"PYTHON freq centroid        : {freq_centroid:.2f}")
    print(f"PYTHON dominant harmonics   : {len(dom_harmonics)}")
    print("==============================================\n")


    #* ======= Classification ==========
    mode = 1

    con = "pending"
    con1 = "Distance 10cm; Height 10cm;"
    con2 = "Distance 10cm; Height 30cm;"
    con3 = "Distance 30cm; Height 10cm;"
    con4 = "Distance 30cm; Height 30cm;"

    if (mode == 1):
        # COIN conditions
        if (stm_rises + stm_falls > 4):
            mat = "COIN detected!"

            if ( (old_energy > 3_000_000) or pp > 3100.0):
                con = con2

            elif (1_400_000 < old_energy < 3_000_000):

                if (peak_freq > 2000):
                    con = con1

                elif (peak_freq < 2000):
                    con = con4

            else:
                con = con3

        # ERASER conditions
        else:
            mat = "Eraser detected!"

            if (old_energy > 400_000):

                con = con2
            
            elif (190_000 < old_energy < 400_000):

                if (peak_freq > 350):
                    con = con4
                else:
                    con = con1  
            
            else :
                con = con3

        print(mat)
        print(con)

    elif (mode == 2):

        if (total_energy > 100):
            print("Eraser Detected.")
        else:
            print("No eraser detected.")

