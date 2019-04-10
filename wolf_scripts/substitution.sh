#!/bin/bash
# folder=$1
# for file in $(ag '(SensorBase|ProcessorBase|FrameBase|CaptureBase|FeatureBase|FactorBase|LandmarkBase|StateBlock)List' . -o); do
for file in $(ag 'Ptr\(' . -o); do
    # file=$(echo $file | sed "s/ //g")
    target=$(echo $file | cut -d':' -f 1)
    line=$(echo $file | cut -d':' -f 2)
    subs=$(echo $file | cut -d':' -f 3)
    # echo "$target@$line@$subs"
    # subs_line=${line}s/${subs}/${subs%List}PtrList/gp
    # echo $subs_line
    # sed -n -e $line's/Ptr\( \)*(\( \)*)/\1(\2)/gp' $target
    # sed -n -e $line's/Ptr\( \)*(\( \)*)/\1(\2)/gp' $target
    sed -i -e $line's/(/(/g' $target
done

# for file in $(ag -l -g constraint $folder); do
#     new_file=$(echo $file | sed -e "s/constraint/factor/g")
#     mv $file $new_file
# done