#!/bin/bash

git submodule update --init --recursive
cp /workspaces/cheongeum/.devcontainer/.zshrc ~/.zshrc
git config --global user.name 'mmemcc' && git config --global user.email 'mjk980917@gmail.com' && git config --global --add safe.directory /workspaces/cheongeum