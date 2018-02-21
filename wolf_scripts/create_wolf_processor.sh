#! /bin/bash

# check wether the scripts path environment variable has been defined
WOLF_SCRIPTS_PATH=`echo "${WOLF_SCRIPTS_PATH}"`
if [ -z "${WOLF_SCRIPTS_PATH}" ]
then
  echo "The scripts path environment varibale has not been defined. Please see the wiki documentation for instructions on how to create it."
  exit
fi

NAME=
BASE=

while getopts ":n:b:" OPTION
do
  case $OPTION in
    n)
       NAME=$OPTARG
       ;;
    b)
       BASE=$OPTARG
       ;;
    ?)
       echo "******************************************"  
       echo "Script to generate a WOLF processor class"
       echo "******************************************"
       echo "Options:"
       echo "  -n: Processor name"
       echo "  -b: Processor base class (inheritance)"
       echo ""  
       echo "Example of usage:"
       echo "create_wolf_processor.sh -n processor_example -b processor_tracker" 
       echo ""
       exit
       ;;
  esac
done

# Templates path
TEMPLATES_PATH=${WOLF_SCRIPTS_PATH}/templates

#Lower case naming
if [ $NAME ]
then
  NAME=$(echo $NAME | tr '[:upper:]' '[:lower:]')
  NAME_CAP=$(echo $NAME | tr '[:lower:]' '[:upper:]')
else
  echo "No processor name provided, aborting ..."
  exit
fi
if [ $BASE ]
then
  BASE=$(echo $BASE | tr '[:upper:]' '[:lower:]')
else
  echo "No base processor name provided, aborting ..."
  exit
fi

# Check naming
if ! echo "$NAME" | grep -q "processor_" ;  
then
    NAME=processor_$NAME;
fi
if ! echo "$BASE" | grep -q "processor_" ; 
then
    BASE=processor_$BASE;
fi

# Find bas class files
if find $WOLF_ROOT -name $BASE.h -print -quit | grep -q '^'; 
then
  BASE_H_PATH=$(find "$WOLF_ROOT/src" -name $BASE.h)
  BASE_CPP_PATH=$(find "$WOLF_ROOT/src" -name $BASE.cpp)
else
  echo "Cannot find the BASE class files for processor $BASE."modify CMakeLists.
  exit
fi

#============================================
echo "Creating new class and methods for $NAME derived from $BASE"
echo ""
echo -n "- Generating CPP and H files."
#============================================

# ===== Create Git branch if requested =====

# TODO

#============================================

# ===== Create HEADER and CPP files =====
CLASSNAME="${NAME#processor_}"
CLASSNAME=Processor$(echo "$(echo "$CLASSNAME" | sed 's/.*/\u&/')") 

#Set the processor and class names on the template files
sed 's/header_file/'"${NAME}.h"'/g' "${TEMPLATES_PATH}"/class_template.cpp > "${TEMPLATES_PATH}"/tmp.cpp
sed 's/class_name/'"${CLASSNAME}"'/g' "${TEMPLATES_PATH}"/tmp.cpp > "${TEMPLATES_PATH}"/tmp2.cpp
rm "${TEMPLATES_PATH}"/tmp.cpp
#rm "${TEMPLATES_PATH}"/tmp2.cpp

sed 's/base_header_file/'"${BASE}.h"'/g' "${TEMPLATES_PATH}"/class_template.h > "${TEMPLATES_PATH}"/tmp.h
sed 's/class_name/'"${CLASSNAME}"'/g' "${TEMPLATES_PATH}"/tmp.h > "${TEMPLATES_PATH}"/tmp2.h
sed 's/PROCESSOR_CLASS/'"${NAME_CAP}"'/g' "${TEMPLATES_PATH}"/tmp2.h > "${TEMPLATES_PATH}"/tmp3.h
rm "${TEMPLATES_PATH}"/tmp.h
rm "${TEMPLATES_PATH}"/tmp2.h

# Copy all pure virtual methods to derived class
FuncInBase=$(grep -e " = 0;" -e "=0;" $BASE_H_PATH)
D=";"   #Multi Character Delimiter
FuncList=($(echo $FuncInBase | sed -e 's/'"$D"'/\n/g' | while read line; do echo $line | sed 's/[\t ]/'"$D"'/g'; done))
for (( i = 0; i < ${#FuncList[@]}; i++ )); do
  FuncList[i]=$(echo ${FuncList[i]} | sed -r 's/'"$D"'/ /g')
  FuncList[i]=$(echo ${FuncList[i]} | sed -r 's/'"virtual"'/ /g')
  FuncList[i]=$(echo ${FuncList[i]} | sed -r 's/'"=0"'/ /g')
  FuncList[i]=$(echo ${FuncList[i]} | sed -r 's/'" = 0"'/ /g')  
  TXTH="${FuncList[i]%"${FuncList[i]##*[![:space:]]}"}"
  TXTCPP_3=$(echo $TXTH | sed 's/.*(//g')
  TXTCPP_2=$(echo $TXTH | sed 's/(.*//g' | sed 's/.* //g')
  TXTCPP_1=$(echo $TXTH | sed 's/'"$TXTCPP_2"'.*//g')
 
  # CPP file
  FUNCNAME=${TXT%*\(}
  #echo FUNCNAME
  sed -i "/\[base class inherited methods\]/a ${TXTCPP_1}${CLASSNAME}::${TXTCPP_2}(${TXTCPP_3}\n\{\n\}\n" "${TEMPLATES_PATH}"/tmp2.cpp
  
  # H file
  sed -i "/\[base class inherited methods\]/a \       \ virtual ${TXTH};\n" "${TEMPLATES_PATH}"/tmp3.h
done

# Rename and move files
NAME_H_PATH=$WOLF_ROOT/src/processors/$NAME.h
NAME_CPP_PATH=$WOLF_ROOT/src/processors/$NAME.cpp
mv "${TEMPLATES_PATH}"/tmp3.h "$NAME_H_PATH"
mv "${TEMPLATES_PATH}"/tmp2.cpp "$NAME_CPP_PATH"

echo " Done."
echo " \--> Created $NAME_H_PATH file."
echo " \--> Created $NAME_CPP_PATH file."

#============================================
# ===== Modify CMakeLists.txt =====
#echo ""
#echo -n "- Modifying CMakeLists.txt to include CPP and H files."

#cp "$WOLF_ROOT/src/CMakeLists.txt" "${TEMPLATES_PATH}"/tmp.txt

#echo ""

#LINENUM=$( sed -n '/\[Add generic derived header before this line\]/=' "${TEMPLATES_PATH}"/tmp.txt )

#sed -i '$LINENUMi$LINENUM This is Line 8' FILE

#LINENUM=$( sed -n "Add generic derived header before this line" "${TEMPLATES_PATH}"/tmp.txt )
#echo $LINENUM

echo " Done."

#============================================

# ===== Create gtest =====
#echo ""
#echo -n "- Creating gtest for $NAME."

# TODO Create gtest

#echo " Done."

#============================================


#grep -e ") = 0;" -e ")=0;" $BASE_H_PATH >> "${TEMPLATES_PATH}"/tmp3.h
#sed "s/=0//g" < "${TEMPLATES_PATH}"/tmp3.h > "${TEMPLATES_PATH}"/tmp4.h
#sed "s/ = 0//g" < "${TEMPLATES_PATH}"/tmp4.h > "${TEMPLATES_PATH}"/tmp5.h
#rm "${TEMPLATES_PATH}"/tmp3.h
#rm "${TEMPLATES_PATH}"/tmp4.h


#======




#create the project directory
#if [ -e "$ORIGNAME" ]
#then
#  echo "  ! $ORIGNAME directory already exists, skipping ..."
#else
#  echo "  > Creating $ORIGNAME directory"
#  mkdir $ORIGNAME
#fi  


#create the CMakeLists.txt script file
#echo "# library source files" >> CMakeLists.tmp
#echo "SET(sources ${NAME}.cpp)" >> CMakeLists.tmp
#echo "# application header files" >> CMakeLists.tmp
#echo "SET(headers ${NAME}.h)" >> CMakeLists.tmp
#echo "# locate the necessary dependencies" >> CMakeLists.tmp
#for x in $arr
#do
#  echo "FIND_PACKAGE($x REQUIRED)" >> CMakeLists.tmp
#done
#echo "# add the necessary include directories" >> CMakeLists.tmp
#echo "INCLUDE_DIRECTORIES(.)" >> CMakeLists.tmp
#for x in $arr
#do
#  echo "INCLUDE_DIRECTORIES("'${'"${x}_INCLUDE_DIR"'}'")" >> CMakeLists.tmp
#done
#echo "# create the shared library" >> CMakeLists.tmp
###echo "ADD_LIBRARY(${NAME} SHARED "'${'"sources"'}'")" >> CMakeLists.tmp
#echo "# link necessary libraries" >> CMakeLists.tmp
#for x in $arr
#do
#  echo "TARGET_LINK_LIBRARIES(${NAME} "'${'"${x}_LIBRARY"'}'")" >> CMakeLists.tmp
#done
#echo "INSTALL(TARGETS $NAME" >> CMakeLists.tmp
#echo "        RUNTIME DESTINATION bin" >> CMakeLists.tmp
#echo "        LIBRARY DESTINATION lib/${NAME}" >> CMakeLists.tmp
#echo "        ARCHIVE DESTINATION lib/${NAME})" >> CMakeLists.tmp
#echo "INSTALL(FILES "'${'"headers"'}' "DESTINATION include/${NAME})" >> CMakeLists.tmp
#echo "INSTALL(FILES ../Find$NAME.cmake DESTINATION "'${'"CMAKE_ROOT"'}'"/Modules/)" >> CMakeLists.tmp
#echo "ADD_SUBDIRECTORY(examples)" >> CMakeLists.tmp
#mv CMakeLists.tmp $ORIGNAME/src/CMakeLists.txt
  
#echo "# create an example application" >> CMakeLists.tmp
#echo "ADD_EXECUTABLE(${NAME}_test ${NAME}_test.cpp)" >> CMakeLists.tmp
#echo "# link necessary libraries" >> CMakeLists.tmp
#echo "TARGET_LINK_LIBRARIES(${NAME}_test $NAME)" >> CMakeLists.tmp
#for x in $arr
#do
#  echo "TARGET_LINK_LIBRARIES(${NAME}_test "'${'"${x}_LIBRARY"'}'")" >> CMakeLists.tmp
#done
#mv CMakeLists.tmp $


##Set the project name on the Findlib.cmake file
#sed 's/header_file/'"${NAME}.h"'/g' <$TEMPLATES_PATH/Findlib_template.cmake >tmp.cmake
#sed 's/project_name/'$ORIGNAME'/g' <tmp.cmake >tmp2.cmake
#sed 's/library_name/'$NAME'/g' <tmp2.cmake >$ORIGNAME/Find$NAME.cmake
#rm tmp.cmake
#rm tmp2.cmake

#LIBRARY_NAME=$(echo $NAME | tr '[:lower:]' '[:upper:]')
#Library_name=$(echo $NAME | sed 's/\([a-zA-Z]\)\([a-zA-Z0-9]*\)/\u\1\2/g')
#sed 's/Library_name/'$Library_name'/g' <$TEMPLATES_PATH/library_header_template.h >tmp.h
#sed 's/LIBRARY_NAME/'$LIBRARY_NAME'/g' <tmp.h >$ORIGNAME/src/$NAME.h
#rm tmp.h
  
#sed 's/library_name/'$NAME'/g' <$TEMPLATES_PATH/library_src_template.cpp >tmp.cpp
#sed 's/Library_name/'$Library_name'/g' <tmp.cpp >tmp2.cpp
#sed 's/Library_name/'$Library_name'/g' <tmp2.cpp >$ORIGNAME/src/$NAME.cpp
#rm tmp.cpp
#rm tmp2.cpp

#sed 's/library_name/'$NAME'/g' <$TEMPLATES_PATH/example_src_template.cpp >$ORIGNAME/src/examples/${NAME}_test.cpp

#sed 's/library_name/'$NAME'/g' <$TEMPLATES_PATH/main_template.dox >tmp.dox
#sed 's/project_name/'"$ORIGNAME"'/g' <tmp.dox >$ORIGNAME/doc/main.dox
#rm tmp.dox

##create gitignores
#echo "  > Creating gitignore files"
#if [ -f $BIN_DIR/.gitignore ]
#then
#  echo "    ! gitignore file already exist in bin directory, skipping ..."
#else
#  echo "    > creating .gitignore file in bin directory"	
#  cp $WOLF_SCRIPTS_PATH/cpp_project_template/gitignore_template $BIN_DIR/.gitignore
#fi

#if [ -f $BUILD_DIR/.gitignore ]
#then
#  echo "    ! gitignore file already exist in build directory, skipping ..."
#else
#  echo "    > creating .gitignore file in build directory"	
#  cp $WOLF_SCRIPTS_PATH/cpp_project_template/gitignore_template $BUILD_DIR/.gitignore
#fi

#if [ -f $LIB_DIR/.gitignore ]
#then
#  echo "    ! gitignore file already exist in lib directory, skipping ..."
#else
#  echo "    > creating .gitignore file in lib directory"	
#  cp $WOLF_SCRIPTS_PATH/cpp_project_template/gitignore_template $LIB_DIR/.gitignore
#fi

#echo "Project created."

