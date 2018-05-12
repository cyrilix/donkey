#!/usr/bin/env bash

IMG_DIR=$1
OUTPUT_FILE=$2
FPS=20

if [ -z "${IMG_DIR}" ]
then
    echo "img dir missing"
    exit 1
fi

if [ -z "${OUTPUT_FILE}" ]
then
    echo "output file missing"
    exit 1
fi

ffmpeg -r 25 -pattern_type glob -i "${IMG_DIR}/cam-image_array_*_.jpg" -c:v libx264 -vf "fps=${FPS},format=yuv420p" ${OUTPUT_FILE}
