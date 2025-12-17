#!/bin/bash

# This script is for deploying the Docusaurus site to GitHub Pages.

# 1. Set the current branch to a variable
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

# 2. Deploy to GitHub Pages
GIT_USER=bariqa-anwar USE_SSH=false npm run deploy
