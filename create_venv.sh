#!/usr/bin/env bash
set -e

# Path to the destination virtual environment (defaults to "venv" in the current directory)
DEST_VENV="${1:-venv}"
REQUIREMENTS_FILE="requirements.txt"

if [ ! -f "$REQUIREMENTS_FILE" ]; then
    echo "Error: $REQUIREMENTS_FILE not found in the current directory."
    exit 1
fi

echo "Creating new virtual environment at $DEST_VENV..."
# Using --system-site-packages is recommended if you need access to ROS 2 or system-level Python packages 
# that were filtered out of requirements.txt because they aren't available on PyPI.
python3 -m venv --system-site-packages "$DEST_VENV"

echo "Installing dependencies from $REQUIREMENTS_FILE into $DEST_VENV..."

# Try to install the requirements in bulk first
if ! "$DEST_VENV/bin/pip" install -r "$REQUIREMENTS_FILE"; then
    echo "Warning: Bulk install failed. Attempting to install packages one by one..."
    # Fallback: install line by line, skipping failures
    while IFS= read -r req || [[ -n "$req" ]]; do
        # skip empty lines and comments
        if [[ -n "$req" ]] && [[ ! "$req" =~ ^# ]]; then
            "$DEST_VENV/bin/pip" install "$req" || echo "Warning: Failed to install '$req'. Skipping."
        fi
    done < "$REQUIREMENTS_FILE"
fi

echo "Done! Virtual environment created successfully."
echo "You can activate your new venv with:"
echo "source $DEST_VENV/bin/activate"
