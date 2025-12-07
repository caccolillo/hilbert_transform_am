#!/usr/bin/env python3
"""
AM Demodulator Results Plotter - Fixed Version

Automatically aligns input, output, and message vectors to the same length
to prevent dimension mismatch errors in plots.

Usage:
    python3 plot_am_results_fixed.py
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

def plot_time_domain(indices, am_input, output, message, sample_rate):
    """Plot time domain signals"""
    time = indices / sample_rate * 1000  # Convert to milliseconds
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('AM Demodulator - Time Domain Analysis', fontsize=14, fontweight='bold')
    
    axes[0].plot(time, am_input, 'b-', linewidth=0.5, label='AM Input')
    axes[0].set_ylabel('Amplitude', fontweight='bold')
    axes[0].set_title('Input AM Modulated Signal')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[0].set_xlim([0, min(50, time[-1])])  # Show first 50ms
    
    axes[1].plot(time, output, 'r-', linewidth=1.5, label='Demodulated Output', alpha=0.8)
    axes[1].plot(time, message, 'g--', linewidth=1.5, label='Expected Message', alpha=0.7)
    axes[1].set_ylabel('Amplitude', fontweight='bold')
    axes[1].set_title('Demodulated Output vs Expected Message Signal')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    
    error = output - message
    axes[2].plot(time, error, 'k-', linewidth=0.8, label='Error')
    axes[2].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    axes[2].set_xlabel('Time (ms)', fontweight='bold')
    axes[2].set_ylabel('Error', fontweight='bold')
    axes[2].set_title('Demodulation Error')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()
    
    plt.tight_layout()
    return fig

def plot_frequency_domain(output, message, sample_rate):
    """Plot frequency domain analysis using FFT"""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('AM Demodulator - Frequency Domain Analysis', fontsize=14, fontweight='bold')
    
    N = len(output)
    fft_output = np.fft.fft(output)
    fft_message = np.fft.fft(message)
    freqs = np.fft.fftfreq(N, 1/sample_rate)
    
    pos_mask = freqs >= 0
    freqs_pos = freqs[pos_mask]
    fft_output_pos = np.abs(fft_output[pos_mask])
    fft_message_pos = np.abs(fft_message[pos_mask])
    
    axes[0].semilogy(freqs_pos, fft_output_pos, 'r-', linewidth=1.5, label='Demodulated Output', alpha=0.8)
    axes[0].semilogy(freqs_pos, fft_message_pos, 'g--', linewidth=1.5, label='Expected Message', alpha=0.7)
    axes[0].set_xlabel('Frequency (Hz)', fontweight='bold')
    axes[0].set_ylabel('Magnitude', fontweight='bold')
    axes[0].set_title('FFT Magnitude Spectrum')
    axes[0].grid(True, alpha=0.3, which='both')
    axes[0].legend()
    axes[0].set_xlim([0, min(10000, sample_rate/2)])
    
    f_out, psd_out = signal.welch(output, sample_rate, nperseg=1024)
    f_msg, psd_msg = signal.welch(message, sample_rate, nperseg=1024)
    
    axes[1].semilogy(f_out, psd_out, 'r-', linewidth=1.5, label='Demodulated Output', alpha=0.8)
    axes[1].semilogy(f_msg, psd_msg, 'g--', linewidth=1.5, label='Expected Message', alpha=0.7)
    axes[1].set_xlabel('Frequency (Hz)', fontweight='bold')
    axes[1].set_ylabel('PSD (V²/Hz)', fontweight='bold')
    axes[1].set_title('Power Spectral Density')
    axes[1].grid(True, alpha=0.3, which='both')
    axes[1].legend()
    axes[1].set_xlim([0, min(10000, sample_rate/2)])
    
    plt.tight_layout()
    return fig

def plot_error_statistics(output, message):
    """Plot error statistics and histograms"""
    error = output - message
    error_steady = error[500:]  # Skip transient
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('AM Demodulator - Error Analysis', fontsize=14, fontweight='bold')
    
    axes[0, 0].hist(error_steady, bins=50, color='blue', alpha=0.7, edgecolor='black')
    axes[0, 0].set_xlabel('Error', fontweight='bold')
    axes[0, 0].set_ylabel('Count', fontweight='bold')
    axes[0, 0].set_title('Error Distribution')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axvline(x=0, color='r', linestyle='--', linewidth=2)
    
    cumulative_error = np.cumsum(np.abs(error))
    axes[0, 1].plot(cumulative_error, 'b-', linewidth=1.5)
    axes[0, 1].set_xlabel('Sample Index', fontweight='bold')
    axes[0, 1].set_ylabel('Cumulative |Error|', fontweight='bold')
    axes[0, 1].set_title('Cumulative Absolute Error')
    axes[0, 1].grid(True, alpha=0.3)
    
    axes[1, 0].scatter(output[500::10], error[500::10], alpha=0.3, s=10)
    axes[1, 0].axhline(y=0, color='r', linestyle='--', linewidth=2)
    axes[1, 0].set_xlabel('Output Value', fontweight='bold')
    axes[1, 0].set_ylabel('Error', fontweight='bold')
    axes[1, 0].set_title('Error vs Output Value')
    axes[1, 0].grid(True, alpha=0.3)
    
    axes[1, 1].axis('off')
    
    mean_error = np.mean(error_steady)
    std_error = np.std(error_steady)
    max_error = np.max(np.abs(error_steady))
    rmse = np.sqrt(np.mean(error_steady**2))
    snr = 10 * np.log10(np.mean(message[500:]**2) / np.mean(error_steady**2))
    
    stats_text = f"""
    Error Statistics (after transient):
    
    Mean Error:        {mean_error:.6f}
    Std Deviation:     {std_error:.6f}
    Max Abs Error:     {max_error:.6f}
    RMSE:              {rmse:.6f}
    SNR:               {snr:.2f} dB
    
    Samples Analyzed:  {len(error_steady)}
    """
    
    axes[1, 1].text(0.1, 0.5, stats_text, fontsize=12, family='monospace', verticalalignment='center')
    
    plt.tight_layout()
    return fig

def main():
    input_file = 'am_input.dat'
    output_file = 'am_output.dat'
    message_file = 'message_signal.dat'
    
    files = [input_file, output_file, message_file]
    missing_files = [f for f in files if not os.path.exists(f)]
    
    if missing_files:
        print("Error: Missing data files:")
        for f in missing_files:
            print(f"  - {f}")
        sys.exit(1)
    
    print("Reading data files...")
    indices_in, am_input = read_data_file(input_file)
    indices_out, output = read_data_file(output_file)
    indices_msg, message = read_data_file(message_file)
    
    if am_input is None or output is None or message is None:
        print("Error reading one or more files")
        sys.exit(1)
    
    # Align all arrays to minimum length
    indices_out, am_input, output, message = align_lengths(indices_out, am_input, output, message)
    
    SAMPLE_RATE = 480000.0
    
    print(f"Loaded {len(am_input)} samples")
    
    print("\nGenerating plots...")
    fig1 = plot_time_domain(indices_out, am_input, output, message, SAMPLE_RATE)
    fig2 = plot_frequency_domain(output, message, SAMPLE_RATE)
    fig3 = plot_error_statistics(output, message)
    
    print("\nSaving plots...")
    fig1.savefig('am_demod_time_domain.png', dpi=300, bbox_inches='tight')
    fig2.savefig('am_demod_frequency_domain.png', dpi=300, bbox_inches='tight')
    fig3.savefig('am_demod_error_analysis.png', dpi=300, bbox_inches='tight')
    
    print("\n✓ All plots saved. Displaying...")
    plt.show()

if __name__ == '__main__':
    main()

