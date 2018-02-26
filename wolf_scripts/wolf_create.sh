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
askIfGitBranch $NAME

#============================================
echo "- Generating CPP and H files."
#============================================

# Find base class files
BASE_H_PATH=$(getFilePath $BASE.h)
BASE_CPP_PATH=$(getFilePath $BASE.cpp)

# Create Header and CPP files
createHCPPFromTemplates $NAME_H_PATH $NAME_CPP_PATH

# Copy all pure virtual methods from base class to derived class
fillWithBaseVirtualMethods

echo " \--> Created $NAME_H_PATH file."
echo " \--> Created $NAME_CPP_PATH file."
#============================================

# ===== Modify CMakeLists.txt =====
echo ""
echo "- Modifying CMakeLists.txt to include CPP and H files."

DONE=$(updateCMakeLists)
if [ -z "$DONE" ]
then
  echo " \--x [WARN]: Not necessary. CPP and H files already existing in CMakeLists.txt."
else
  echo " \--> Updated ${DONE} file."
fi

# ===== Create gtest =====
echo ""
echo "- Creating gtest for $NAME."

createGtest
echo " \--> Created ${GTEST_PATH} file."

#============================================

# ===== Modify gtest CMakeLists.txt =====

echo ""
echo "- Modifying CMakeLists.txt to include gtest files."

DONE=$(updateCMakeListsGTest)
if [ -z "$DONE" ]
then
  echo " \--x [WARN]: Not necessary. gtest file already existing in CMakeLists.txt."
else
  echo " \--> Updated ${DONE} file."
fi

#============================================

echo "All Done."
echo ""
#============================================