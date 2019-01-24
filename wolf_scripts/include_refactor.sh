#!/bin/bash
# First parameter $1 is the path to the wolf root
# Second parameter $2 is the name of the new folder

hdrs_folder=$1/include/base/$2
srcs_folder=$1/src/$2
#Fix the includes in general
core_headers=$(ag -g .*\\.h -l $1/include/base/$2/ | rev | cut -d '/' -f 1 | rev)
for f in $core_headers; do
    ch_files=$(ag \#include[[:space:]]+\"\(.*\/\)*"$2/$f"\" $1/include/base/ -l)
    for fp in $ch_files; do
        # echo ".h -> "$fp
        sed -i -E "s/(#include[[:space:]]+\")("$2"\/)?"$f"\"/\1base\/"$2"\/"$f"\"/gp" $fp
        sed -i -E "s/(#include[[:space:]]+<)("$2"\/)?"$f">/\1base\/"$2"\/"$f">/gp" $fp
    done
    cs_files=$(ag \#include[[:space:]]+\"\(.*\/\)*"$2/$f"\" $1/src/ -l)
    for fp in $cs_files; do
        # echo ".cpp -> "$fp
        sed -i -E "s/(#include[[:space:]]+\")("$2"\/)?"$f"\"/\1base\/"$2"\/"$f"\"/gp" $fp
        sed -i -E "s/(#include[[:space:]]+<)("$2"\/)?"$f">/\1base\/"$2"\/"$f">/gp" $fp
    done
done