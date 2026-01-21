import numpy as np
import scipy.signal as signal
import soundfile as sf
import matplotlib.pyplot as plt
from tkinter import Tk
from tkinter.filedialog import askopenfilename

# GUI to select WAV file
Tk().withdraw()
filename = askopenfilename(title="Select WAV file", filetypes=[("WAV files", "*.wav")])
if not filename:
    print("No file selected.")
    exit()

# Load the audio file
audio, Fs = sf.read(filename)
if audio.ndim > 1:
    audio = audio[:, 0]  # Convert stereo to mono

# FIR Filter Design: Bandpass 300–3400 Hz with 70 taps
order = 299  # 70 taps = order + 1
bands = [0, 200, 300, 3400, 4000, Fs/2]  # In Hz (not normalized)
desired = [0, 1, 0]  # Desired gain per band

# Design FIR filter using Parks-McClellan (remez)
b = signal.remez(order + 1, bands, desired, fs=Fs)

# Apply the filter
filtered = signal.lfilter(b, 1, audio)

# Signal Power (mean squared)
power_before = np.mean(audio**2)
power_after = np.mean(filtered**2)
print(f"\nSignal Power BEFORE filtering: {power_before:.6f}")
print(f"Signal Power AFTER filtering:  {power_after:.6f}")

# Save filtered audio
sf.write("filtered_output.wav", filtered, Fs)
print("\n Filtered audio saved as 'filtered_output.wav'.")

# Plot time-domain waveforms
t = np.arange(len(audio)) / Fs
plt.figure(figsize=(12, 6))

plt.subplot(2, 1, 1)
plt.plot(t, audio, label="Original", color='blue')
plt.title("Original Audio Waveform")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")

plt.subplot(2, 1, 2)
plt.plot(t, filtered, label="Filtered", color='orange')
plt.title("Filtered Audio Waveform")
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")

plt.tight_layout()
plt.show()

# Frequency Spectrum Comparison
def plot_spectrum(sig, Fs, label):
    N = len(sig)
    freqs = np.fft.rfftfreq(N, 1/Fs)
    spectrum = np.abs(np.fft.rfft(sig) / N)
    plt.plot(freqs, 20 * np.log10(spectrum + 1e-10), label=label)  # Add epsilon to avoid log(0)

plt.figure(figsize=(12, 5))
plot_spectrum(audio, Fs, "Original")
plot_spectrum(filtered, Fs, "Filtered")
plt.title("Frequency Spectrum (Before vs After Filtering)")
plt.xlabel("Frequency (Hz)")
plt.xlim(0,1/250)
plt.ylabel("Magnitude (dB)")
plt.legend()
plt.grid(True)
plt.tight_layout()
#plt.show()
