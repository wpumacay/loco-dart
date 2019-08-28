#!/bin/sh

# --------------------------------------------------------------------- #
# script/setup: Sets up this repo by cloning all required dependencies  #
# --------------------------------------------------------------------- #

for repo in ext/dart ext/cat1 ext/imgui ext/pybind11 core
do
    echo "Fetching: ${repo}"
    cd "${repo}" 
    git fetch
    cd "../.."
    echo "----------------------------------------------------"
done