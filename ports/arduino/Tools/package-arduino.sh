#!/usr/bin/env bash
set -euo pipefail

# Usage: ./release.sh 1.0.1
VERSION="${1:-dev}"
RELEASE_ROOT="release"
ARDUINO_DIR="$RELEASE_ROOT/arduino"

# Clean slate
rm -rf "$RELEASE_ROOT"
mkdir -p "$ARDUINO_DIR/src"
mkdir -p "$ARDUINO_DIR/examples"

echo "📦 Packaging SmoothAxis version: $VERSION"

# --- 1. Copy Core (Flattened) ---
# We put these directly in src/ so SmoothAxis.h can find them easily
cp src/smooth_axis.c "$ARDUINO_DIR/src/"
cp src/smooth_axis.h "$ARDUINO_DIR/src/"
# cp src/smooth_axis_debug.h "$ARDUINO_DIR/src/" # Optional

# --- 2. Copy Wrapper ---
# Note: Ensure SmoothAxis.h uses #include "smooth_axis.h", not "../../src/..."
cp ports/arduino/SmoothAxis.h "$ARDUINO_DIR/src/"

# --- 3. Copy Metadata & Examples ---
cp -r ports/arduino/examples/* "$ARDUINO_DIR/examples/"
cp ports/arduino/keywords.txt "$ARDUINO_DIR/"
cp ports/arduino/library.properties "$ARDUINO_DIR/"

# --- 4. Documentation ---
cp LICENSE "$ARDUINO_DIR/"
# Optional: Create a specific README for Arduino users if the main one is too technical
cp README.md "$ARDUINO_DIR/"

# --- 5. Version Sync (The Magic Trick) ---
# Updates "version=x.x.x" in library.properties to match the tag
if [ "$VERSION" != "dev" ]; then
    if [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS sed syntax
        sed -i '' "s/^version=.*/version=$VERSION/" "$ARDUINO_DIR/library.properties"
    else
        # Linux sed syntax
        sed -i "s/^version=.*/version=$VERSION/" "$ARDUINO_DIR/library.properties"
    fi
    echo "✅ Updated library.properties to $VERSION"
fi

# --- 6. Zip It ---
(
  cd "$RELEASE_ROOT"
  zip -r "smooth_axis_arduino_${VERSION}.zip" arduino/
)

echo "🎉 Done! Artifact: $RELEASE_ROOT/smooth_axis_arduino_${VERSION}.zip"