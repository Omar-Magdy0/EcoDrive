#!/bin/bash

# Use matching versions
ARM_VERSION="15.2.rel1"
ARM_DIR="arm-gnu-toolchain-${ARM_VERSION}-x86_64-arm-none-eabi"
ARM_PATH="/opt/${ARM_DIR}"
ARM_URL="https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_VERSION}/binrel/${ARM_DIR}.tar.xz"

# Always update and install apt packages
sudo apt update
sudo apt install -y cmake build-essential xz-utils socat

# Check if ARM toolchain already exists
if [ -d "$ARM_PATH" ]; then
    echo "ARM toolchain ${ARM_VERSION} already installed at ${ARM_PATH}"
else
    echo "ARM toolchain not found. Downloading and installing..."
    wget -O /tmp/arm-toolchain.tar.xz $ARM_URL
    sudo tar -xf /tmp/arm-toolchain.tar.xz -C /opt
    rm /tmp/arm-toolchain.tar.xz
fi

# Ensure PATH is set
echo 'export PATH=$PATH:'"$ARM_PATH"'/bin' | sudo tee /etc/profile.d/arm-toolchain.sh
sudo chmod +x /etc/profile.d/arm-toolchain.sh
export PATH=$PATH:$ARM_PATH/bin

echo "ARM toolchain ${ARM_VERSION} ready"

sudo apt update
sudo apt install libboost-system-dev libboost-thread-dev libboost-date-time-dev

#Install Libglfw3 and opengl if needed
sudo apt install libglfw3-dev libgl1-mesa-dev libhdf5-dev
sudo apt-get install libusb-1.0-0-dev
