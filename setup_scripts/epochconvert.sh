#!/bin/bash

# Check if an epoch time argument is provided
if [ $# -ne 1 ]; then
    echo "Usage: $0 <epoch_time>"
    exit 1
fi

epoch_time="$1"

# Convert epoch time to UTC ISO format
utc_date=$(date -u -j -f %s "$epoch_time" 2>/dev/null)

if [ $? -ne 0 ]; then
    echo "Error: Invalid epoch time."
    exit 1
fi

# Convert epoch time to local ISO format
local_date=$(date -j -f %s "$epoch_time" 2>/dev/null)

if [ $? -ne 0 ]; then
    echo "Error: Invalid epoch time."
    exit 1
fi

echo "UTC: $utc_date"
echo "Local: $local_date"

