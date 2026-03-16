#!/bin/bash

# Use matching versions
ARM_VERSION="15.2.rel1"
ARM_DIR="arm-gnu-toolchain-${ARM_VERSION}-x86_64-arm-none-eabi"
ARM_PATH="/opt/${ARM_DIR}"
ARM_URL="https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_VERSION}/binrel/${ARM_DIR}.tar.xz"

# Always update and install apt packages
sudo apt update
sudo apt install -y cmake build-essential xz-utils socat python3 python3-pip python3.12-venv python3-full

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

echo "Creating Virtual Environment for EcoTool..."
cd tools

# Create virtual environment properly
echo "Creating new virtual environment..."
python3 -m venv env

# Check if venv creation was successful
if [ ! -f "env/bin/activate" ]; then
    echo "ERROR: Failed to create virtual environment"
    echo "Trying alternative method with explicit Python version..."
    python3.12 -m venv env
fi

# Verify activation script exists
if [ -f "env/bin/activate" ]; then
    echo "Virtual environment created successfully"
    
    # Activate virtual environment
    echo "Activating virtual environment..."
    source env/bin/activate
    
    # Upgrade pip first
    echo "Upgrading pip..."
    pip install --upgrade pip
    
    # Install requirements
    if [ -f "requirements.txt" ]; then
        echo "Installing requirements from requirements.txt..."
        pip install --ignore-requires-python pyqtdarktheme==2.1.0
        pip install -r requirements.txt
        echo "Requirements installed successfully"
    else
        echo "Warning: requirements.txt not found in tools directory"
    fi
    
    echo "======================================"
    echo "Virtual environment is now active!"
    echo "To activate it manually later, run: source tools/env/bin/activate"
    echo "Current Python: $(which python)"
    echo "Current pip: $(which pip)"
else
    echo "ERROR: Could not create virtual environment"
    echo "Please try manually:"
    echo "  sudo apt install python3.12-venv"
    echo "  cd tools"
    echo "  python3 -m venv env"
    echo "  source env/bin/activate"
    echo "  pip install -r requirements.txt"
    exit 1
fi

#Install Libglfw3 and opengl if needed
sudo apt install libglfw3-dev libgl1-mesa-dev