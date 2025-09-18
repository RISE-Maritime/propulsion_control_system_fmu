#!/bin/bash
set -e

# -------- CONFIG --------
MODEL_NAME="PropulsionControlSystem"
FMU_NAME="PropulsionControlSystem.fmu"
BUILD_MODE="release"
BUILD_DIR="target"
OUTPUT_DIR="dist/fmu_temp"
MODEL_DESCRIPTION="modelDescription.xml"
# ------------------------

# Step 1: Ensure cross is installed
if ! command -v cross &> /dev/null; then
    echo "Installing cross..."
    cargo install cross
fi

# Step 2: Compile locally to generate modelDescription.xml
echo "Generating modelDescription.xml..."
cargo build --release

# Step 2: Cross-compiling for linux and windows
echo "Building shared libraries..."
cross build --release --target x86_64-unknown-linux-gnu
cross build --release --target x86_64-pc-windows-gnu

# Step 3: Locate binaries and modelDescription.xml
LINUX_LIB_SRC="$BUILD_DIR/x86_64-unknown-linux-gnu/$BUILD_MODE/lib${MODEL_NAME}.so"
LINUX_LIB_DST="${MODEL_NAME}.so"
WIN_LIB_SRC="$BUILD_DIR/x86_64-pc-windows-gnu/$BUILD_MODE/${MODEL_NAME}.dll"
MODEL_DESC_PATH="$MODEL_DESCRIPTION"

if [[ ! -f "$LINUX_LIB_SRC" ]]; then
    echo "Linux shared library not found: $LINUX_LIB_SRC"
    exit 1
fi

if [[ ! -f "$WIN_LIB_SRC" ]]; then
    echo "Windows shared library not found: $WIN_LIB_SRC"
    exit 1
fi

if [[ ! -f "$MODEL_DESC_PATH" ]]; then
    echo "modelDescription.xml not found!"
    exit 1
fi

# Step 5: Create FMU folder structure
echo "Creating FMU structure..."
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR/binaries/x86_64-linux"
mkdir -p "$OUTPUT_DIR/binaries/x86_64-windows"
cp "$LINUX_LIB_SRC" "$OUTPUT_DIR/binaries/x86_64-linux/$LINUX_LIB_DST"
cp "$WIN_LIB_SRC" "$OUTPUT_DIR/binaries/x86_64-windows/"
cp "$MODEL_DESC_PATH" "$OUTPUT_DIR/"

# Optional: include resources or documentation if needed
# mkdir -p "$OUTPUT_DIR/resources"
# mkdir -p "$OUTPUT_DIR/docs"

# Step 6: Zip into FMU
echo "Packaging into FMU..."
pushd "$OUTPUT_DIR" > /dev/null
zip -r "../$FMU_NAME" *
popd > /dev/null

echo "FMU created at: $BUILD_DIR/$FMU_NAME"