### NAOqi Python SDK Setup (macOS Guide)

This guide documents the specific steps required to run the Aldebaran/SoftBank Robotics Python SDK (v2.5.x) on a modern macOS environment using pyenv and a custom alias to avoid breaking the system's main Python.

1. Prerequisites
   Python 2.7.18 installed via pyenv.
   NAOqi SDK folder (e.g., naoqi-sqk) located on the Desktop.
   Apple Silicon Macs: VS Code (or your Terminal) must be running in Rosetta mode to handle the Intel-based SDK binaries.
2. Shell Configuration (~/.zshrc)
   Add these lines to the bottom of your shell profile to create a dedicated command for the robot without affecting your default python3 environment.
   bash

# Custom alias to trigger the Robot environment

alias python-local='/Users/nicholascarducci/.pyenv/versions/2.7.18/bin/python'

# SDK Path (Update if folder name changes)

export PYTHONPATH="$PYTHONPATH:/Users/nicholascarducci/Desktop/naoqi-sqk/lib/python2.7/site-packages"

# Binary Library Path

export DYLD_LIBRARY_PATH="$DYLD_LIBRARY_PATH:/Users/nicholascarducci/Desktop/naoqi-sqk/lib"
Use code with caution.

3. Essential Security & System Fixes
   These commands solved the "Permission Denied" and "Library Not Loaded" errors.
   A. Bypass macOS Gatekeeper
   Remove the quarantine flag from the SDK folder (required because the files were transferred/downloaded):
   bash
   sudo xattr -rd com.apple.quarantine ~/Desktop/naoqi-sqk
   Use code with caution.

B. Python Framework Symlink
The SDK has a hardcoded path looking for Python in the system library. We created a "shortcut" to point it to the pyenv version:
bash

# Create the directory structure

sudo mkdir -p /Library/Frameworks/Python.framework/Versions/2.7/

# Link the pyenv library to the expected system path

sudo ln -sf /Users/nicholascarducci/.pyenv/versions/2.7.18/lib/libpython2.7.dylib /Library/Frameworks/Python.framework/Versions/2.7/Python
Use code with caution.

4. Running Scripts
   Always use the python-local command to run your robot code.
   bash
   python-local your_script.py
   Use code with caution.

5. Troubleshooting Checklist
   "No module named naoqi": Ensure your PYTHONPATH in ~/.zshrc points exactly to the site-packages folder inside the SDK.
   "Library not loaded": Verify the symlink in /Library/Frameworks/Python.framework/Versions/2.7/Python is correctly pointing to your .pyenv path.
   "Incompatible architecture": Right-click VS Code in your Applications folder -> Get Info -> Check "Open using Rosetta".
