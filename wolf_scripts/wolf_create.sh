#! /bin/bash

# Load functions
. $WOLF_SCRIPTS_PATH/generic_func/functions.sh

# check environment variables are been defined
WOLF_ROOT=$(getEnvVariable WOLF_ROOT)
WOLF_SCRIPTS_PATH=$(getEnvVariable WOLF_SCRIPTS_PATH)
if [ -z "${WOLF_ROOT}" ] || [ -z "${WOLF_SCRIPTS_PATH}" ] ;
then 
	exit 
fi

# Source user menu to obtain main variables
. $WOLF_SCRIPTS_PATH/generic_func/user_menu.sh

#============================================
echo ""
echo "==========================================================================================================================="
echo "     Creating new class and methods for $NAME derived from $BASE"
echo "==========================================================================================================================="
echo ""
#============================================

# Create git branch is requested
#askIfGitBranch $NAME

#============================================
echo -n "- Generating CPP and H files."
#============================================

# Find base class files
BASE_H_PATH=$(getFilePath $BASE.h)
BASE_CPP_PATH=$(getFilePath $BASE.cpp)

# Create Header and CPP files
createHCPPFromTemplates $NAME_H_PATH $NAME_CPP_PATH

# Copy all pure virtual methods from base class to derived class
fillWithBaseVirtualMethods

echo " Done."
echo " \--> Created $NAME_H_PATH file."
echo " \--> Created $NAME_CPP_PATH file."
#============================================

# ===== Modify CMakeLists.txt =====
echo ""
echo -n "- Modifying CMakeLists.txt to include CPP and H files."

updateCMakeLists

echo " Done."
echo " \--> Updated $CML_PATH file."

# ===== Create gtest =====
#echo ""
#echo -n "- Creating gtest for $NAME."

# TODO Create gtest

#echo " Done."

#============================================

echo "All Done."
echo ""
#============================================




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

