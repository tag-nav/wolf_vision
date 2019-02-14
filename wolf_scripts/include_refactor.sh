#!/bin/bash
for ff in $(find ~/workspace/wip/wolf/templinks/ -follow | cut -d '/' -f 8- | grep ".h$\|.cpp$"); do
    for f in $(cat ~/workspace/wip/wolf/files.txt); do
        path=$(ag -g /$f$ -l ~/workspace/wip/wolf/ | cut -d '/' -f 8-)
        matches=$(echo $path | wc -w)
        if [ $matches -gt 1 ]; then
            # echo $f " -> " $path
            path=$(echo $path | cut -d ' ' -f 1)
        fi
        # echo $f " now in -> " $path " modifying file "$ff
        # sed -i -E "s:(#include[[:space:]]+)."$f".:\1\""$path"\":gp" ~/workspace/wip/wolf/$ff
        sed -i -E "s:(#include[[:space:]]+).(\.\.\/)+(.+\/)+"$f".:\1\""$path"\":g" ~/workspace/wip/wolf/$ff
    done
done
# for f in $(cat ~/workspace/wip/wolf/files.txt); do
#     path=$(ag -g /$f$ -l ~/workspace/wip/wolf/ | cut -d '/' -f 7-)
#     matches=$(echo $path | wc -w)
#     if [ $matches -gt 1 ]; then
#         echo $f " -> " $(echo $path | cut -d ' ' -f 1)
#     fi
# done