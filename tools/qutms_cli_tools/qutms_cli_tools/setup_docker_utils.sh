#!/bin/bash

# Define file names
SRC_ALIAS_FILE="qutms_aliases"
ALIAS_FILE=".qutms_aliases"
BASHRC="$HOME/.bashrc"

echo "Step 1: Copying $ALIAS_FILE to $HOME..."
if [ -f "$SRC_ALIAS_FILE" ]; then
    cp "$SRC_ALIAS_FILE" "$HOME/$ALIAS_FILE"
    echo "Successfully copied $ALIAS_FILE to home directory."
else
    echo "Error: $SRC_ALIAS_FILE not found in the current directory."
    exit 1
fi

echo "Step 2: Registering aliases in $BASHRC..."

# Check if the loading block is already present to avoid duplicates
if grep -q "Load QUTMS aliases" "$BASHRC"; then
    echo "QUTMS aliases are already registered in $BASHRC."
else
    echo -e "\n# Load QUTMS aliases\nif [ -f ~/$ALIAS_FILE ]; then\n    . ~/$ALIAS_FILE\nfi" >> "$BASHRC"
    echo "Added alias loading logic to $BASHRC."
fi

echo "Step 3: Sourcing $BASHRC..."
# Note: Sourcing inside a script only affects the script's subshell.
# The user will still need to run 'source ~/.bashrc' in their parent shell.
source "$BASHRC"

echo "Registration complete! Please run 'source ~/.bashrc' or restart your terminal to activate the aliases."
