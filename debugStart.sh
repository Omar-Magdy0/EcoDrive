#!/bin/bash

# BMP Debug Script for gdbgui + gdb-multiarch
# Usage: ./debug_bmp.sh

set -e  # Exit on any error
gdbgui -g "gdb-multiarch -x gdb_commands.gdb" --args EcoDrive/build/bin/EcoDrive.elf