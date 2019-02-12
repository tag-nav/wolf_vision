#!/bin/bash
# $files=$(ag "^[^\\#].*(\\.h$|\\.cpp$)" CMakeCompact.txt | cut -d ':' -f 2 | sed "s/^/\//g" | rev | cut -d '/' -f 1 | rev)
# echo $files
for f in $(cat ~/workspace/wip/wolf/files.txt); do
    path=$(ag -g $f -l ~/workspace/wip/wolf/ | cut -d '/' -f 7-)
    # echo $f " now in -> " $path
    sed -i "/"$f"$/c\\"$path ~/workspace/wip/wolf/CMakeLists.txt
done