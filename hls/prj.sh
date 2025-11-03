#!/bin/bash

# Script name: run_all_hls_builds.sh
# Description: Recursively find and run all hls_build.sh scripts in subdirectories

echo "Searching for hls_build.sh in subdirectories..."

# Find and execute hls_build.sh in each subdirectory
find . -type f -name "hls_build.sh" | while read script; do
    dir=$(dirname "$script")
    echo "Running hls_build.sh in: $dir"
    (cd "$dir" && chmod +x hls_build.sh && ./hls_build.sh)
done

echo "All hls_build.sh scripts have been executed."
