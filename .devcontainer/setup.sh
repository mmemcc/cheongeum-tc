#!/bin/bash


cp /workspaces/cheongeum/.devcontainer/.zshrc ~/.zshrc
git config --global user.name 'mmemcc' && git config --global user.email 'mjk980917@gmail.com'
git config --global --add safe.directory /workspaces/cheongeum

# idf.py
export IDF_PATH=/workspaces/cheongeum/esp-idf
export PATH="$IDF_PATH/tools:$PATH"
export PATH="$IDF_PATH/tools/idf.py:$PATH"
