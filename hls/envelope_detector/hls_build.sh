#!/bin/bash

# Script name: run_vitis_hls.sh

# Exit on any error
set -e

echo "Starting Vitis HLS build process..."

# Run the first Vitis HLS command
echo "Running: vitis_hls hls_build.tcl"
vitis_hls hls_build.tcl

# Run the second Vitis HLS command
#echo "Running: vitis_hls -p filter_envelope_detector/hls.app"
#vitis_hls -p filter_envelope_detector/hls.app

echo "Vitis HLS build completed successfully."

