#!/bin/bash

set -e

echo "Updating package list..."
sudo apt update

echo "Installing GPIO Zero..."
sudo apt install -y python3-gpiozero python3-lgpio

echo "Testing GPIO Zero..."
python3 -c "import gpiozero; print('GPIO Zero installed successfully:', gpiozero.__version__)"

echo "Installing the required python packages..."
source .venv/bin/activate
pip install -r requirements.text


echo "Done!"
