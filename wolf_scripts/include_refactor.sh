#!/bin/bash
for folder in problem hardware trajectory map frame state_block common math utils; do
    for ff in $(find include/base/$folder src/$folder -type f); do
        name=$(echo $ff | rev | cut -d '/' -f 1 | rev)
        old="base/$name"
        new="base/$folder/$name"
        # echo "%%%%%%%%% "$ff " ¬¬ $name"
        # echo "$old ºº $new"
        # for target in $(find include/base src test -type f); do
        for target in $(find hello_wolf -type f); do
            # out=$(sed -E -n "s:$old:$new:gp" $target)
            out=$(sed -i -E "s:$old:$new:g" $target)
            if [[ $out ]]; then
                echo ">>> changing : $old -> $new @ $target"
                echo $out
            fi
        done
    done
done

# for ff in $(find ~/workspace/wip/wolf/templinks/ -follow | cut -d '/' -f 8- | grep ".h$\|.cpp$"); do
#     for f in $(cat ~/workspace/wip/wolf/files.txt); do
#         path=$(ag -g /$f$ -l ~/workspace/wip/wolf/ | cut -d '/' -f 8-)
#         matches=$(echo $path | wc -w)
#         if [ $matches -gt 1 ]; then
#             # echo $f " -> " $path
#             path=$(echo $path | cut -d ' ' -f 1)
#         fi
#         # echo $f " now in -> " $path " modifying file "$ff
#         # sed -i -E "s:(#include[[:space:]]+)."$f".:\1\""$path"\":gp" ~/workspace/wip/wolf/$ff
#         sed -i -E "s:(#include[[:space:]]+).(\.\.\/)+(.+\/)+"$f".:\1\""$path"\":g" ~/workspace/wip/wolf/$ff
#     done
# done
# for f in $(cat ~/workspace/wip/wolf/files.txt); do
#     path=$(ag -g /$f$ -l ~/workspace/wip/wolf/ | cut -d '/' -f 7-)
#     matches=$(echo $path | wc -w)
#     if [ $matches -gt 1 ]; then
#         echo $f " -> " $(echo $path | cut -d ' ' -f 1)
#     fi
# done