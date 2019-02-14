#!/bin/bash
for folder in capture constraint core examples feature landmark processor sensor; do
    # out=$(find ~/workspace/wip/wolf/src/$folder -type f | rev | cut -d '/' -f 1 | rev)
    out=$(find ~/workspace/wip/wolf/src/$folder -type f)
    for file in $out; do
        for prefix in capture constraint core feature landmark processor sensor; do
            if [ "$folder" == "$prefix" ]; then
                sed -i -E "s@(#include[[:space:]]+\")\.\./include/"$prefix/"(.*)\"@\1\2\"@gp" $file
                # echo +====================== $file ==================================
                # sed -n -E "s@(#include[[:space:]]+\")\.\./include/"$prefix/"(.*)\"@\1\2\"@gp" $file
                # echo -===============================================================
            fi
        done
    done
done
for folder in capture constraint core examples feature landmark processor sensor; do
    out=$(find ~/workspace/wip/wolf/src/$folder -type f)
    for file in $out; do
        f=$(echo $file | rev | cut -d '/' -f 1 | rev)
        mv $file ~/workspace/wip/wolf/src/$folder/$f
        # echo $file " -----> "~/workspace/wip/wolf/src/$folder/$f
        sed -i -E "s:.*"$f"$:"$folder/$f":p" ~/workspace/wip/wolf/src/CMakeLists.txt
    done
done