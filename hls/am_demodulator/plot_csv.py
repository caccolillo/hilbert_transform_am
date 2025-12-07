#!/usr/bin/env python3
"""
AM Demodulator Results Plotter - Full Version with SINAD and Last 10% Window
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import sys
import os

def read_data_file(filename):
    """Read data file with format: index value"""
    try:
        data = np.loadtxt(filename)
        indices = data[:, 0].astype(int)
        values = data[:, 1]
        return indices, values
    except Exception as e:
        print(f"Error reading {filename}: {e}")
        return None, None

def align_lengths(*arrays):
    """Align multiple arrays to the minimum length"""
    min_len = min(len(a) for a in arrays)
    return tuple(a[:min_len] for a in arrays)

def compute_sinad(signal_vec):
    """Compute SINAD (Signal-to-Noise and Distortion Ratio)."""
    x = signal_vec - np.mean(signal_vec)
    N = len(x)
    spectrum = np.fft.fft(x)
    mag = np.abs(spectrum[:N // 2]) ** 2

    fundamental_bin = np.argmax(mag[1:]) + 1
    signal_power = mag[fundamental_bin]
    noise_power = np.sum(mag) - signal_power

    return 10 * np.log10(signal_power / noise_power)

def plot_time_domain(indices, am_input, output, message, sample_rate):
    """Plot time domain signals"""
    time = indices / sample_rate * 1000

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('AM Demodulator - Time Domain Analysis', fontsize=14, fontweight='bold')

    axes[0].plot(time, am_input, linewidth=0.5, label='AM Input')
    axes[0].set_ylabel('Amplitude')
    axes[0].set_title('Input AM Modulated Signal')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[0].set_xlim([0, min(50, time[-1])])

    axes[1].plot(time, output, linewidth=1.2, label='Demodulated Output')
    axes[1].plot(time, message, '--', linewidth=1.2, label='Expected Message')
    axes[1].set_ylabel('Amplitude')
    axes[1].set_title('Demodulated Output vs Expected Message Signal')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    error = output - message
    axes[2].plot(time, error, linewidth=0.8, label='Error')
    axes[2].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    axes[2].set_xlabel('Time (ms)')
    axes[2].set_ylabel('Error')
    axes[2].set_title('Demodulation Error')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    plt.tight_layout()
    return fig

def plot_last_section_and_sinad(last_section, sample_rate, sinad_value):
    """Open a second window showing the last 10% of samples and SINAD."""
    N = len(last_section)
    t = np.arange(N) / sample_rate * 1000

    x = last_section - np.mean(last_section)
    spectrum = np.fft.fft(x)
    mag = 20 * np.log10(np.abs(spectrum[:N // 2]) + 1e-12)
    freqs = np.fft.fftfreq(N, d=1 / sample_rate)[:N // 2]

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle("Last 10% Samples + SINAD Analysis", fontsize=14, fontweight='bold')

    axes[0].plot(t, last_section, linewidth=1.0)
    axes[0].set_title("Last 10% of Output Signal (Time Domain)")
    axes[0].set_xlabel("Time (ms)")
    axes[0].set_ylabel("Amplitude")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(freqs, mag, linewidth=1.0)
    axes[1].set_title("Last 10% FFT (Frequency Domain)")
    axes[1].set_xlabel("Frequency (Hz)")
    axes[1].set_ylabel("Magnitude (dB)")
    axes[1].grid(True, alpha=0.3)

    fig.text(0.5, 0.02, f"SINAD: {sinad_value:.2f} dB", ha="center", fontsize=12)

    plt.tight_layout()
    return fig

def main():
    input_file = 'am_input.dat'
    output_file = 'am_output.dat'
    message_file = 'message_signal.dat'

    files = [input_file, output_file, message_file]
    missing = [f for f in files if not os.path.exists(f)]
    if missing:
        print("Error: Missing data files:")
        for f in missing:
            print("  -", f)
        sys.exit(1)

    indices_in, am_input = read_data_file(input_file)
    indices_out, output = read_data_file(output_file)
    indices_msg, message = read_data_file(message_file)

    if am_input is None or output is None or message is None:
        print("Error reading one or more files")
        sys.exit(1)

    indices_out, am_input, output, message = align_lengths(indices_out, am_input, output, message)
    SAMPLE_RATE = 480000.0

    print(f"Loaded {len(am_input)} samples")

    last_section = output[int(len(output) * 0.9):]
    sinad_value = compute_sinad(last_section)
    print(f"\nSINAD (last 10% of samples): {sinad_value:.2f} dB\n")

    fig1 = plot_time_domain(indices_out, am_input, output, message, SAMPLE_RATE)
    fig1.text(0.5, 0.01, f"SINAD (last 10%): {sinad_value:.2f} dB", ha='center', fontsize=12)
    fig1.savefig("am_demod_time_domain.png", dpi=300, bbox_inches='tight')

    fig2 = plot_last_section_and_sinad(last_section, SAMPLE_RATE, sinad_value)
    fig2.savefig("am_demod_last_section_sinad.png", dpi=300, bbox_inches='tight')

    plt.show()

if __name__ == '__main__':
    main()
