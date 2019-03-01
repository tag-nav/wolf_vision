#!/bin/bash
# $1 path to the root of the plugin
# $2 name of the plugin
# $3 files to be moved
# Create the target directory
# if [ -d $1 ];
# then
#     rm -rf $1
# fi

#Generate the necessary dirs
if [ ! -d $1/include/$2 ];
then
    mkdir $1/include/$2
fi
for folder in capture constraint feature landmark processor sensor ; do
    if [ ! -d $1/include/$2/$folder ];
    then
        mkdir $1/include/$2/$folder
    fi
    if [ ! -d $1/src/$folder ];
    then
        mkdir $1/src/$folder
    fi
done
for file in $(cat $3); do
    head=$(echo $file | cut -d '/' -f 1)
    if [ "$head" = "include" ];
    then
        folder=$(echo $file | cut -d '/' -f 3)
        suffix=$(echo $file | cut -d '/' -f 4-)
        line=$(ag "HDRS_"${folder^^} $1/CMakeLists.txt | cut -d ':' -f 1 | head -1)
        line=$(($line + 1))
        echo $line " && " $file " && " $folder " && " $suffix
        sed  -i ""$line"i $head/$2/$folder/$suffix" $1/CMakeLists.txt
        cp $file $1/$head/$2/$folder/$suffix
    else
        folder=$(echo $file | cut -d '/' -f 2)
        suffix=$(echo $file | cut -d '/' -f 3-)
        ag "SRCS_"$folder $1/CMakeLists.txt
        line=$(ag "SRCS_"${folder^^} $1/CMakeLists.txt | cut -d ':' -f 1 | head -1)
        line=$(($line + 1))
        echo $line " && " $file " && " $folder " && " $suffix
        sed  -i ""$line"i $file" $1/CMakeLists.txt
        cp $file $1/$head/$folder/$suffix
    fi
done
for f in $(cat $3); do
    hhead=$(echo $f | cut -d '/' -f 1)
    if [ "$hhead" = "include" ];
    then
        ffolder=$(echo $f | cut -d '/' -f 3)
        ssuffix=$(echo $f | cut -d '/' -f 4-)
        inc=$ffolder/$ssuffix
    else
        continue
    fi
    for ff in $(cat $3); do
        head=$(echo $ff | cut -d '/' -f 1)
        if [ "$head" = "include" ];
        then
            folder=$(echo $ff | cut -d '/' -f 3)
            suffix=$(echo $ff | cut -d '/' -f 4-)
            new_path=$1/$head/$2/$folder/$suffix
            # sed -n -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@pg" $new_path
            sed -i -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@g" $new_path
        else
            folder=$(echo $ff | cut -d '/' -f 2)
            suffix=$(echo $ff | cut -d '/' -f 3-)
            new_path=$1/$head/$folder/$suffix
            # sed -n -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@pg" $new_path
            sed -i -E "s@(#include[[:space:]]+\")base(\/$inc\")@\1$2\2@g" $new_path
        fi
    done
done