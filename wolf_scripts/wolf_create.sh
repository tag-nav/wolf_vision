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
echo "${BLUE}     Creating new class and methods for $NAME derived from $BASE ${NC}"
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
if [ -z "$BASE_H_PATH" ]
then
  echo ""
  echo "${RED}  [ERROR]: Cannot find header file for base class ${BASE}.${NC}"
  echo ""
  exit
fi

# Create Header and CPP files
createHCPPFromTemplates $NAME_H_PATH $NAME_CPP_PATH

# Copy all pure virtual methods from base class to derived class
fillWithBaseVirtualMethods

echo "${GREEN} \--> Created $NAME_H_PATH file.${NC}"
echo "${GREEN} \--> Created $NAME_CPP_PATH file.${NC}"
#============================================

# ===== Modify CMakeLists.txt =====
echo ""
echo "- Modifying CMakeLists.txt to include CPP and H files."

DONE=$(updateCMakeLists)
if [ -z "$DONE" ]
then
  echo "${YELLOW} \--x [WARN]: Not necessary. File entries already existing in CMakeLists.txt.${NC}"
else
  echo "${GREEN} \--> Updated ${DONE} file.${NC}"
fi

# ===== Create gtest =====
echo ""
echo "- Creating gtest for $NAME."

createGtest

echo "${GREEN} \--> Created ${GTEST_PATH} file.${NC}"

#============================================

# ===== Modify gtest CMakeLists.txt =====

echo ""
echo "- Modifying CMakeLists.txt to include gtest files."

DONE=$(updateCMakeListsGTest)
if [ -z "$DONE" ]
then
  echo "${YELLOW} \--x [WARN]: Not necessary. gtest file entry already existing in CMakeLists.txt.${NC}"
else
  echo "${GREEN} \--> Updated ${DONE} file.${NC}"
fi

#============================================

echo ""
echo "All Done."
echo ""
#============================================