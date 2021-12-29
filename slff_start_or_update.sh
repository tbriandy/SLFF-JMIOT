#!/bin/bash
set -e

REPO_PATH="$HOME/SLFF-App-Docker"
REPO_URL="https://github.com/protanjung/SLFF-App-Docker.git"

# If the repo not exists, clone it
# Else, pull the latest changes
# ================================
if [ ! -d "$REPO_PATH" ]; then
    git clone $REPO_URL $REPO_PATH
else
    cd $REPO_PATH
    git pull
fi

# Enter the repo
# ==============
cd $REPO_PATH

# Docker compose pull and up
# ==========================
docker-compose pull
docker-compose up --detach --remove-orphans

# Docker image prune
# ==================
docker image prune --all --force