import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, iirnotch, resample
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
from scipy.signal import butter, filtfilt, iirnotch, resample, find_peaks
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean

# Paths for both files
path_nrf = 'D:\\AAA_PERSONAL\\COM ENGG 24-25 S2\\COE 199\\sir ron code\\rust_reader\\nrf_100s_3.csv'
path_ganglion = 'D:\\AAA_PERSONAL\\COM ENGG 24-25 S2\\COE 199\\sir ron code\\ganglion_100s_3.csv'

# Sampling rates
fs_nrf = 250
fs_ganglion = 200
fs_target = 200

# Load NRF signal
df_nrf = pd.read_csv(path_nrf)
signal_nrf = pd.to_numeric(df_nrf.iloc[:, 1], errors='coerce').dropna()

# Load Ganglion signal
df_ganglion = pd.read_csv(path_ganglion)
signal_ganglion = df_ganglion['3'] / 1e6

# Filtering helpers
def butter_highpass_filter(data, cutoff=0.05, fs=fs_target, order=1):
    nyq = 0.5 * fs
    b, a = butter(order, cutoff / nyq, btype='high')
    return filtfilt(b, a, data)

def apply_notch_filter(data, notch_freq=60.0, fs=fs_target, quality=30):
    w0 = notch_freq / (fs / 2)
    b, a = iirnotch(w0, Q=quality)
    return filtfilt(b, a, data)

def butter_lowpass_filter(data, cutoff=20, fs=fs_target, order=3):
    nyq = 0.5 * fs
    b, a = butter(order, cutoff / nyq, btype='low')
    return filtfilt(b, a, data)

def filter_ecg(signal):
    s = butter_highpass_filter(signal, cutoff=1)
    s = apply_notch_filter(s)
    s = butter_lowpass_filter(s, cutoff=20)
    return s

# Resample NRF to 200 Hz
num_samples_target = int(len(signal_nrf) * fs_target / fs_nrf)
signal_nrf_resampled = resample(signal_nrf, num_samples_target)

# Truncate to match lengths (keep max length for alignment)
min_len = min(len(signal_nrf_resampled), len(signal_ganglion))
signal_nrf_resampled = signal_nrf_resampled[:min_len]
signal_ganglion = signal_ganglion[:min_len]

# Filter both
filtered_nrf = filter_ecg(signal_nrf_resampled)
filtered_ganglion = filter_ecg(signal_ganglion)



# === Cross-Correlation Alignment ===
cross_corr = np.correlate(filtered_nrf - np.mean(filtered_nrf),
                          filtered_ganglion - np.mean(filtered_ganglion), mode='full')
lag_idx = np.argmax(cross_corr) - (len(filtered_nrf) - 1)
lag_time = lag_idx / fs_target
print(f"\n== Cross-Correlation Alignment ==\nLag Index: {lag_idx}, Lag Time: {lag_time:.4f} s")

# Shift NRF to align with Ganglion
if lag_idx > 0:
    aligned_nrf = filtered_nrf[lag_idx:]
    aligned_ganglion = filtered_ganglion[:len(aligned_nrf)]
elif lag_idx < 0:
    aligned_ganglion = filtered_ganglion[-lag_idx:]
    aligned_nrf = filtered_nrf[:len(aligned_ganglion)]
else:
    aligned_nrf = filtered_nrf
    aligned_ganglion = filtered_ganglion

# Time vector after alignment
t = np.arange(len(aligned_nrf)) / fs_target

# === Correlation Coefficient After Alignment ===
correlation_after = np.corrcoef(aligned_nrf, aligned_ganglion)[0, 1]
print(f"\n== Post-Alignment Correlation ==\nPearson r = {correlation_after:.4f}")

# === RMSE After Alignment ===
rmse = np.sqrt(np.mean((aligned_nrf - aligned_ganglion)**2))
print(f"RMSE = {rmse:.6f}")
normalized_rmse = rmse / (np.max(aligned_ganglion) - np.min(aligned_ganglion))
print(f"Normalized RMSE = {normalized_rmse:.4%}")

# Define time window in seconds
start_time = 4
end_time = 6

# Find indices for the time window
indices = np.where((t >= start_time) & (t <= end_time))[0]

# Slice signals for that window
nrf_window = aligned_nrf[indices]
ganglion_window = aligned_ganglion[indices]

# === Correlation Coefficient in the 4-6s Window ===
correlation_window = np.corrcoef(nrf_window, ganglion_window)[0, 1]
print(f"\n== Correlation (4-6 s Window) ==\nPearson r = {correlation_window:.4f}")

# === RMSE in the 4-6s Window ===
rmse_window = np.sqrt(np.mean((nrf_window - ganglion_window)**2))
print(f"RMSE (4-6 s) = {rmse_window:.6f}")

# Normalize RMSE by amplitude range of Ganglion signal in window
norm_rmse_window = rmse_window / (np.max(ganglion_window) - np.min(ganglion_window))
print(f"Normalized RMSE (4-6 s) = {norm_rmse_window:.4%}")


from scipy.signal import find_peaks

def compute_heart_rate(signal, fs, min_distance_sec=0.4):
    """
    Detect R-peaks and compute average heart rate in BPM.
    - signal: filtered ECG signal
    - fs: sampling frequency (Hz)
    - min_distance_sec: minimum time between peaks (refractory period) to avoid false detection
    """
    min_distance_samples = int(min_distance_sec * fs)
    # Find peaks (R peaks are highest points in ECG)
    peaks, _ = find_peaks(signal, distance=min_distance_samples, height=np.mean(signal) + 0.5 * np.std(signal))
    if len(peaks) < 2:
        return None, peaks  # Not enough peaks to estimate HR
    
    # Calculate RR intervals in seconds
    rr_intervals = np.diff(peaks) / fs
    mean_rr = np.mean(rr_intervals)
    heart_rate_bpm = 60 / mean_rr
    return heart_rate_bpm, peaks

def percent_difference(hr1, hr2):
    return abs(hr1 - hr2) / ((hr1 + hr2) / 2) * 100


# Optionally compute heart rate after alignment for the aligned signals
hr_nrf_aligned, peaks_nrf_aligned = compute_heart_rate(aligned_nrf, fs_target)
hr_ganglion_aligned, peaks_ganglion_aligned = compute_heart_rate(aligned_ganglion, fs_target)

print(f"Estimated Heart Rate NRF (after alignment): {hr_nrf_aligned:.2f} BPM")
print(f"Estimated Heart Rate Ganglion (after alignment): {hr_ganglion_aligned:.2f} BPM")

# Assuming hr_nrf_aligned and hr_ganglion_aligned are heart rates after alignment:
if hr_nrf_aligned is not None and hr_ganglion_aligned is not None:
    diff_after = percent_difference(hr_nrf_aligned, hr_ganglion_aligned)
    print(f"Percent Difference in Heart Rate (after alignment): {diff_after:.2f}%")


# Plot
plt.figure(figsize=(12, 6))
plt.plot(t, aligned_nrf, label='NRF')
plt.plot(t, aligned_ganglion, label='Ganglion', alpha=0.7)
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.title('Filtered ECG Signals (NRF vs Ganglion) - Aligned by Cross-Correlation')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
