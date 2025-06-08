import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, iirnotch, resample, correlate
from scipy.stats import pearsonr
from sklearn.metrics import mean_squared_error

# Paths for both files
path_nrf = 'D:\\AAA_PERSONAL\\COM ENGG 24-25 S2\\COE 199\\sir ron code\\rust_reader\\nrf_emg_100s_6.csv'
path_ganglion = 'D:\\AAA_PERSONAL\\COM ENGG 24-25 S2\\COE 199\\sir ron code\\ganglion_emg_100s_6.csv'

# Sampling rates
fs_nrf = 250
fs_ganglion = 200
fs_target = 200  # Resample NRF to this

# Filter functions
def butter_highpass_filter(data, cutoff=1, fs=fs_target, order=1):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return filtfilt(b, a, data)

def apply_notch_filter(data, notch_freq=60.0, fs=fs_target, quality=30):
    w0 = notch_freq / (fs / 2)
    b_notch, a_notch = iirnotch(w0, Q=quality)
    return filtfilt(b_notch, a_notch, data)

def butter_lowpass_filter(data, cutoff=50, fs=fs_target, order=3):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return filtfilt(b, a, data)

# Load and clean NRF
df_nrf = pd.read_csv(path_nrf)
signal_nrf = pd.to_numeric(df_nrf.iloc[:, 1], errors='coerce').dropna().values
signal_nrf_resampled = resample(signal_nrf, int(len(signal_nrf) * fs_target / fs_nrf))

# Load Ganglion
df_ganglion = pd.read_csv(path_ganglion)
signal_ganglion = df_ganglion['3'].dropna().values.astype(float) / 1e6

# Trim to match length
min_len = min(len(signal_nrf_resampled), len(signal_ganglion))
signal_nrf = signal_nrf_resampled[:min_len]
signal_ganglion = signal_ganglion[:min_len]

# Apply filters
signal_nrf_filtered = butter_lowpass_filter(
    butter_highpass_filter(apply_notch_filter(signal_nrf)), cutoff=50
)
signal_ganglion_filtered = butter_lowpass_filter(
    butter_highpass_filter(apply_notch_filter(signal_ganglion)), cutoff=50
)

# Calculate Mean Absolute Value (MAV)
def mean_absolute_value(signal):
    return np.mean(np.abs(signal))

mav_nrf = mean_absolute_value(signal_nrf_filtered)
mav_ganglion = mean_absolute_value(signal_ganglion_filtered)

print(f"MAV (nRF): {mav_nrf:.6f}")
print(f"MAV (Ganglion): {mav_ganglion:.6f}")

# Absolute difference
abs_diff = abs(mav_nrf - mav_ganglion)

# Percentage difference relative to Ganglion MAV (or average)
percent_diff = abs_diff / ((mav_nrf + mav_ganglion) / 2) * 100

# Ratio (nRF divided by Ganglion)
ratio = mav_nrf / mav_ganglion if mav_ganglion != 0 else np.nan

print(f"Absolute Difference: {abs_diff:.6f}")
print(f"Percentage Difference: {percent_diff:.2f}%")
print(f"MAV Ratio (nRF / Ganglion): {ratio:.4f}")

rms_nrf = np.sqrt(np.mean(signal_nrf_filtered**2))
rms_ganglion = np.sqrt(np.mean(signal_ganglion_filtered**2))

print(f"RMS NRF: {rms_nrf:.6f}")
print(f"RMS Ganglion: {rms_ganglion:.6f}")

# Comparison similar to MAV
abs_diff_rms = abs(rms_nrf - rms_ganglion)
percent_diff_rms = abs_diff_rms / ((rms_nrf + rms_ganglion) / 2) * 100
print(f"RMS Absolute Difference: {abs_diff_rms:.6f}")
print(f"RMS Percentage Difference: {percent_diff_rms:.2f}%")

# Peak-to-Peak Amplitude
ptp_nrf = np.ptp(signal_nrf_filtered)
ptp_ganglion = np.ptp(signal_ganglion_filtered)

# Standard Deviation
std_nrf = np.std(signal_nrf_filtered)
std_ganglion = np.std(signal_ganglion_filtered)

# Output
print(f"Peak-to-Peak NRF: {ptp_nrf:.6f}")
print(f"Peak-to-Peak Ganglion: {ptp_ganglion:.6f}")
print(f"STD NRF: {std_nrf:.6f}")
print(f"STD Ganglion: {std_ganglion:.6f}")

# Time axis
time = np.linspace(0, min_len / fs_target, min_len)
# Plot
plt.figure(figsize=(12, 6))
plt.plot(time, signal_nrf_filtered, label='nRF EMG', linewidth=1)
plt.plot(time, signal_ganglion_filtered, label='Ganglion EMG', linewidth=1, alpha=0.7)
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.title('Filtered EMG Signal Comparison (60s)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
