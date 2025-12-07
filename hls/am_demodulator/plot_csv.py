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
    
    print("\nSaving plots...")
    fig1.savefig('am_demod_time_domain.png', dpi=300, bbox_inches='tight')

    
    print("\n✓ All plots saved. Displaying...")
    plt.show()

if __name__ == '__main__':
    main()

