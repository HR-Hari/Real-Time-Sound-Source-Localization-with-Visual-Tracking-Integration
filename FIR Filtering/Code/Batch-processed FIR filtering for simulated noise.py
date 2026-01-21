import numpy as np
import scipy.io.wavfile as wav
import scipy.signal as signal
import matplotlib.pyplot as plt
from tkinter import Tk
from tkinter.filedialog import askopenfilename
from matplotlib.backends.backend_pdf import PdfPages
import csv
import os

# ---------------- File selection ----------------
Tk().withdraw()
filename = askopenfilename(title="Select WAV file", filetypes=[("WAV files", "*.wav")])
if not filename:
    print("No file selected.")
    exit()

# ---------------- Load WAV ----------------
Fs, data = wav.read(filename)
if data.dtype == np.int16:
    data = data / 32768.0
elif data.dtype == np.int32:
    data = data / 2147483648.0

s = data[:, 0] if data.ndim == 2 else data

# ---------------- FIR Filter Design ----------------
lowcut = 950
highcut = 1050
numtaps = 181
b = signal.firwin(numtaps, [lowcut, highcut], pass_zero=False, fs=Fs)

# ---------------- Setup for plots and output ----------------
results = []
pdf_filename = "filtered_signals.pdf"
csv_filename = "snr_results.csv"

# Focus on a short time window for clarity in plots (3 cycles of 1 kHz tone)
cycles = 3
freq = 1000
samples_to_plot = int((cycles / freq) * Fs)
t_plot = np.arange(samples_to_plot) / Fs

with PdfPages(pdf_filename) as pdf:
    for SNR_dB in range(60, -25, -5):
        # --- Add white Gaussian noise ---
        signal_power = np.mean(s ** 2)
        noise_power = signal_power / (10 ** (SNR_dB / 10))
        noise = np.random.normal(0, np.sqrt(noise_power), size=s.shape)
        x = s + noise

        # --- Filtering ---
        filtered_clean = signal.filtfilt(b, 1, s)
        filtered_noisy = signal.filtfilt(b, 1, x)

        # --- Residual and SNR calculation ---
        residual = filtered_noisy - filtered_clean
        filtered_snr = 10 * np.log10(np.mean(filtered_clean**2) / np.mean(residual**2))
        noise_power_out = np.mean(residual**2)
        filtered_signal_power = np.mean(filtered_clean**2)

        # --- Store results ---
        results.append([
            SNR_dB,
            filtered_snr,
            noise_power_out,
            filtered_signal_power,
            signal_power,
            noise_power
        ])

        # --- Plot (auto-scaled) ---
        plt.figure(figsize=(10, 4))
        plt.plot(t_plot, x[:samples_to_plot], label="Noisy", alpha=0.5)
        plt.plot(t_plot, filtered_noisy[:samples_to_plot], label="Filtered", alpha=0.8)
        plt.plot(t_plot, s[:samples_to_plot], label="Clean", alpha=0.7)
        plt.title(f"SNR In: {SNR_dB} dB → Out: {filtered_snr:.2f} dB")
        plt.xlabel("Time (s)")
        plt.ylabel("Amplitude")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        pdf.savefig()
        plt.close()

# ---------------- Save CSV with footer ----------------
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        "Original SNR (dB)",
        "Filtered SNR (dB)",
        "Residual Noise Power",
        "Filtered Signal Power",
        "Original Signal Power",
        "Added Noise Power"
    ])
    writer.writerows(results)

    file.write("\n")
    file.write("# Notes:\n")
    file.write("# - Residual Noise Power = mean((Filtered_Noisy - Filtered_Clean)^2)\n")
    file.write("# - Filtered Signal Power = mean(Filtered_Clean^2)\n")
    file.write("# - Original Signal Power = mean(Clean^2)\n")
    file.write("# - Added Noise Power = theoretical noise added before filtering\n")
    file.write("# - FIR filter: 950–1050 Hz, zero-phase\n")
    file.write("# - Plots show 3 cycles of 1kHz tone with auto-scaled axes for clarity\n")

# ---------------- Done ----------------
print(f"Saved CSV to: {os.path.abspath(csv_filename)}")
print(f"Saved plots to: {os.path.abspath(pdf_filename)}")
