#!/usr/bin/env python3
"""
Plot AXI Stream data from logfile.txt (simple hex format)
Converts ap_fixed<16,4> format to real values and plots the waveform
"""

import matplotlib.pyplot as plt
import numpy as np

def read_hex_file(filename):
    """
    Read hex values from file and convert to ap_fixed<16,4> real values
    
    Args:
        filename: Path to the hex data file
        
    Returns:
        numpy array of real values
    """
    data = []
    
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                # Read hex value
                hex_val = int(line, 16)
                
                # Convert to signed 16-bit integer
                if hex_val >= 0x8000:
                    signed_val = hex_val - 0x10000
                else:
                    signed_val = hex_val
                
                # Convert from ap_fixed<16,4> to real (divide by 2^12 = 4096)
                real_val = signed_val / 4096.0
                data.append(real_val)
    
    return np.array(data)

def plot_waveform(data, filename='logfile.txt'):
    """
    Plot the waveform data
    
    Args:
        data: numpy array of samples
        filename: name of input file (for title)
    """
    plt.figure(figsize=(12, 6))
    
    # Create sample index
    samples = np.arange(len(data))
    
    # Plot the waveform
    plt.plot(samples, data, linewidth=0.5, color='blue')
    plt.grid(True, alpha=0.3)
    plt.xlabel('Sample Index')
    plt.ylabel('Amplitude (ap_fixed<16,4>)')
    plt.title(f'AXI Stream Data from {filename}')
    
    # Add statistics to the plot
    stats_text = f'Samples: {len(data)}\n'
    stats_text += f'Min: {np.min(data):.3f}\n'
    stats_text += f'Max: {np.max(data):.3f}\n'
    stats_text += f'Mean: {np.mean(data):.3f}\n'
    stats_text += f'Std: {np.std(data):.3f}'
    
    plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes,
             verticalalignment='top', bbox=dict(boxstyle='round', 
             facecolor='wheat', alpha=0.5), fontfamily='monospace')
    
    plt.tight_layout()
    plt.show()

def main():
    filename = 'logfile.txt'
    
    try:
        print(f"Reading data from {filename}...")
        data = read_hex_file(filename)
        print(f"Successfully read {len(data)} samples")
        
        print("\nFirst 10 samples:")
        for i in range(min(10, len(data))):
            print(f"  [{i}] = {data[i]:.6f}")
        
        print("\nPlotting waveform...")
        plot_waveform(data, filename)
        
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
