#!/bin/bash

# Exit on any error
set -e

echo "Starting Vivado build process..."
echo "Running: prj.tcl"
vivado -mode batch -source prj.tcl

echo "Vivado build completed successfully."

