#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
tag=latest

docker build -f $SCRIPT_DIR/../Dockerfile -t ga58lar/openlidarmap:$tag $SCRIPT_DIR/..
