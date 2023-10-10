#!/bin/bash

# Get the current date and time components in UTC
YEAR=$(date -u +"%y")
MONTH=$(date -u +"%m")
DAY=$(date -u +"%d")
DAY_OF_WEEK=$(date -u +"%u") # 1 = Monday, 7 = Sunday
HOURS=$(date -u +"%H")
MINUTES=$(date -u +"%M")
SECONDS=$(date -u +"%S")

# Convert DAY_OF_WEEK to 1-7 format (1 = Sunday)
if [ $DAY_OF_WEEK -eq 7 ]; then
  DAY_OF_WEEK=1
else
  DAY_OF_WEEK=$((DAY_OF_WEEK + 1))
fi

# Zero-pad all fields except DAY_OF_WEEK
YEAR=$(printf "%02d" $YEAR)
DAY=$(printf "%02d" $DAY)
HOURS=$(printf "%02d" $HOURS)
SECONDS=$(printf "%02d" $SECONDS)

# Generate the formatted string
DATE_TIME="${YEAR}${MONTH}${DAY}${DAY_OF_WEEK}${HOURS}${MINUTES}${SECONDS}x"

# Print the result
echo $DATE_TIME

