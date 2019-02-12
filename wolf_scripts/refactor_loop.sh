#!/bin/bash
for folder in capture core constraint feature internal landmark processor sensor temp ; do
    ./include_refactor.sh ~/workspace/wip/wolf $folder
done