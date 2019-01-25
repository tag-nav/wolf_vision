#!/bin/bash
# First parameter $1 is the path to the wolf root
# Second parameter $2 is the name of the new folder
# Third parameter $3 is the path to the folder containing the files

#Generate the necessary dirs
if [ ! -d $1/src/$2 ];
then
    mkdir $1/src/$2
fi
if [ ! -d $1/include/base/$2 ];
then
    mkdir $1/include/base/$2
fi
hdrs_folder=$1/include/base/$2
srcs_folder=$1/src/$2
#Move the .h files
hdrs=$(ag -l -g .*\\.h $3 | rev | cut -d '/' -f 1 | rev)
for fl in $hdrs; do
    echo $fl
    mv $3/$fl $hdrs_folder/$fl
done
#Move the .cpp files
srcs=$(ag -l -g .*\\.cpp $3 | rev | cut -d '/' -f 1 | rev)
for fl in $srcs; do
    echo $fl
    mv $3/$fl $srcs_folder/$fl
done