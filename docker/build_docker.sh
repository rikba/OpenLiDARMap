#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
tag=0.1.0

docker build -f $SCRIPT_DIR/../Dockerfile -t ga58lar/openlidarmap:$tag $SCRIPT_DIR/..
