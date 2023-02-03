#!/usr/bin/env bash

set -e

# Set current working directory to the directory of the script
cd "$(dirname "$0")"

cd ../last_letter/external/last_letter_lib/models

# Generate the model folders in the user home
./skywalker_2013.py

