#!/bin/bash
# PetaLinux Build Script for Ultra96
# Required files in parent directory:
#   - bd_wrapper.xsa
#   - config
#   - rootfs_config
#   - petalinuxbsp.conf
#   - wired.network
#   - system-user.dtsi (optional)

set -e  # Exit on error

PROJECT_NAME="ultra96"

echo "=== Creating PetaLinux Project ==="
petalinux-create --type project --template zynqMP --name ${PROJECT_NAME}
cd ${PROJECT_NAME}

echo "=== Importing Hardware Description ==="
petalinux-config --get-hw-description ../bd_wrapper.xsa --silentconfig

echo "=== Copying Configuration Files ==="
# Main system configuration
rm -f ./project-spec/configs/config
cp ../config ./project-spec/configs/config

# RootFS configuration (package selections)
if [ -f ../rootfs_config ]; then
    cp ../rootfs_config ./project-spec/configs/rootfs_config
    echo "✓ Copied rootfs_config"
else
    echo "⚠ Warning: rootfs_config not found, using defaults"
fi

# BSP configuration
cp ../petalinuxbsp.conf ./project-spec/meta-user/conf/petalinuxbsp.conf

# Network configuration
mkdir -p ./project-spec/configs/systemd-conf
cp ../wired.network ./project-spec/configs/systemd-conf/wired.network

# Device tree overlay (optional)
if [ -f ../system-user.dtsi ]; then
    mkdir -p ./project-spec/meta-user/recipes-bsp/device-tree/files
    cp ../system-user.dtsi ./project-spec/meta-user/recipes-bsp/device-tree/files/
    echo " Copied system-user.dtsi"
else
    echo " Warning: system-user.dtsi not found, skipping"
fi

echo "=== Applying Custom Configurations ==="
# Re-run config to apply the copied configuration files
petalinux-config --silentconfig

echo "=== Building Project ==="
petalinux-build

echo "=== Building SDK ==="
petalinux-build --sdk

echo "=== Packaging Sysroot ==="
petalinux-package --sysroot

echo "=== Creating Boot Image ==="
cd images/linux
petalinux-package --boot --fsbl zynqmp_fsbl.elf --u-boot u-boot.elf --pmufw pmufw.elf --fpga system.bit

echo ""
echo "=== Build Complete ==="
echo "Boot files located in: images/linux/"
echo "  - BOOT.BIN"
echo "  - image.ub"
echo "  - boot.scr"
echo "  - rootfs.tar.gz"
echo ""
echo "=== SD Card Deployment ==="
echo "To write to SD card:"
echo "  1. Identify your SD card: sudo fdisk -l"
echo "  2. Write rootfs: sudo dd if=./rootfs.ext4 of=/dev/sdX2 bs=4M status=progress"
echo "  3. Copy BOOT.BIN, image.ub, boot.scr to boot partition"
echo ""
echo " WARNING: Double-check device name before using dd command!"
echo ""
echo "=== Testing in QEMU (optional) ==="
echo "To test in QEMU, run from project root:"
echo "  cd .."
echo "  petalinux-boot --qemu --prebuilt 3"
