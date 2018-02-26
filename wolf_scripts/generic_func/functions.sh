#! /bin/bash

# ===== FUNCTIONS =====

askIfGitBranch()
{
 read -p "- Do you want to create a new git branch(Y/n)?  " ANSWER
 DOIT=${ANSWER:-y}
 OK=0
 
 while [ $OK -eq 0 ];
 do
   case "$ANSWER" in
     "y" | "Y")
 	   DOIT="y"
 	   OK=1
 	   ;;
     "n" | "N")
       DOIT="n"
       OK=1
       ;; 
     *)
       OK=0
       echo "Incorrect answer. Please type y or n"
       exit
       ;;
  esac
done   

if [ "$DOIT" == "y" ];
then
  if [ -z "$1" ]
  then
    echo "No argument supplied"
  fi
  cd $WOLF_ROOT 
  git checkout -b $1
  echo " \--> Created git branch $1"
fi
echo ""    
}

getEnvVariable()
{
  local var=`echo "${!1}"`
  if [ -z "${var}" ]
  then
    echo "The environment variable ${1} has not been defined. Please see the wiki documentation for instructions on how to create it." >&2
    exit
  fi
  echo $var
}

showHelp() 
{
  echo "******************************************"  
  echo "Script to generate a WOLF $1 class"
  echo "******************************************"
  echo "Options:"
  echo "  -t: type"
  echo "  -n: name"
  echo "  -b: base class (inheritance)"
  echo ""  
  echo "Example of usage:"
  echo "wolf_create.sh -t processor -n processor_example -b processor_tracker" 
  echo ""
}

LowerCase()
{
	if [ $1 ]
	then
  		echo $1 | tr '[:upper:]' '[:lower:]'
	else
  		echo "No $1 provided, aborting ..."
  		exit
	fi
}

UpperCase()
{
	if [ $1 ]
	then
  		echo $1 | tr '[:lower:]' '[:upper:]'
	else
  		echo "No $1 provided, aborting ..."
  		exit
	fi
}

UpperCaseFirstLetter()
{
  echo "$(echo "$1" | sed 's/.*/\u&/')"
}

getFilePath()
{
  if find $WOLF_ROOT -name "${1}" -print -quit | grep -q '^'; 
  then
    echo "$(find "$WOLF_ROOT/src" -name ${1} )"
  else
    echo "Cannot find $1 file." >&2
    exit
  fi
}

createHCPPFromTemplates()
{
  # Templates path
  TEMPLATES_PATH=${WOLF_SCRIPTS_PATH}/templates

  # ===== Create HEADER and CPP files =====

  #Set the TYPE and class names on the template files
  sed 's/header_file/'"${NAME}.h"'/g' "${TEMPLATES_PATH}"/class_template.cpp > "${TEMPLATES_PATH}"/tmp.cpp
  sed 's/class_name/'"${CLASSNAME}"'/g' "${TEMPLATES_PATH}"/tmp.cpp > "${TEMPLATES_PATH}"/tmp2.cpp
  rm "${TEMPLATES_PATH}"/tmp.cpp

  sed 's/base_header_file/'"${BASE}.h"'/g' "${TEMPLATES_PATH}"/class_template.h > "${TEMPLATES_PATH}"/tmp.h
  sed 's/name_cap/'"${NAME_CAP}"'/g' "${TEMPLATES_PATH}"/tmp.h > "${TEMPLATES_PATH}"/tmp2.h
  sed 's/class_name/'"${CLASSNAME}"'/g' "${TEMPLATES_PATH}"/tmp2.h > "${TEMPLATES_PATH}"/tmp3.h
  sed 's/base_class/'"${BASECLASSNAME}"'/g' "${TEMPLATES_PATH}"/tmp3.h > "${TEMPLATES_PATH}"/tmp4.h
  rm "${TEMPLATES_PATH}"/tmp.h
  rm "${TEMPLATES_PATH}"/tmp2.h
  rm "${TEMPLATES_PATH}"/tmp3.h
  
  # Rename and move files
  NAME_H_PATH="$WOLF_ROOT"/src/"$TYPE"s/"$NAME".h
  NAME_CPP_PATH="$WOLF_ROOT"/src/"$TYPE"s/"$NAME".cpp
  mv "${TEMPLATES_PATH}"/tmp4.h "$NAME_H_PATH"
  mv "${TEMPLATES_PATH}"/tmp2.cpp "$NAME_CPP_PATH"
}

fillWithBaseVirtualMethods()
{
  # Get base class	
  H_TXT=$(cat $BASE_H_PATH)
  H_TXT="${H_TXT##*class $BASECLASSNAME :}"
  BASECLASS_TXT="${H_TXT%%\n\};*}"

  echo "class $BASECLASSNAME :$BASECLASS_TXT };" > "${WOLF_SCRIPTS_PATH}"/class.h

  # H file Get Virtual function declarations with help
  sed -e '/./{H;$!d;}' -e 'x;/ = 0;/!d;' "${WOLF_SCRIPTS_PATH}"/class.h > "${WOLF_SCRIPTS_PATH}"/tmp.h
  sed -r 's/'" = 0"'//g' "${WOLF_SCRIPTS_PATH}"/tmp.h > "${WOLF_SCRIPTS_PATH}"/tmp2.h
  sed -i -e "/\[base class inherited methods\]/r ${WOLF_SCRIPTS_PATH}/tmp2.h" "$NAME_H_PATH" 
  rm "${WOLF_SCRIPTS_PATH}"/tmp.h	
  rm ${WOLF_SCRIPTS_PATH}/tmp2.h
    
  # CPP file  
  FuncInBase=$(grep -e " = 0;" -e "=0;" "${WOLF_SCRIPTS_PATH}/class.h")
  D=";"   #Multi Character Delimiter
  FuncList=($(echo $FuncInBase | sed -e 's/'"$D"'/\n/g' | while read line; do echo $line | sed 's/[\t ]/'"$D"'/g'; done))

  for (( idx = $((${#FuncList[@]}-1)); idx > -1; idx-- )); do
  	TMP=$(echo ${FuncList[idx]} | sed -r 's/'"$D"'/ /g')
    TMP=$(echo $TMP | sed -r 's/'"virtual"'/ /g')
    TMP=$(echo $TMP | sed -r 's/'"=0"'/ /g')
    TMP=$(echo $TMP | sed -r 's/'" = 0"'/ /g')
    TXTH=${TMP%$TMP##*[![:space:]]}
    TXTCPP_3=$(echo $TXTH | sed 's/.*(//g')
    TXTCPP_2=$(echo $TXTH | sed 's/(.*//g' | sed 's/.* //g')
    TXTCPP_1=$(echo $TXTH | sed 's/'"$TXTCPP_2"'.*//g')
 
    # CPP file
    sed -i "/\[base class inherited methods\]/a ${TXTCPP_1}${CLASSNAME}::${TXTCPP_2}(${TXTCPP_3}\n\{\n\}\n" "$NAME_CPP_PATH"  
  done    
    
  rm "${WOLF_SCRIPTS_PATH}"/class.h
}

updateCMakeLists()
{	
  CML_PATH="${WOLF_ROOT}/src/${TYPE}s/CMakeLists.txt"

  # Add Header source
  Hsources=( $(grep -e ".h" "${CML_PATH}") )
  NewH="\${CMAKE_CURRENT_SOURCE_DIR}/$NAME.h"
  
  # Check if already exists
  EXISTS=0
  for (( idx = 0; idx < ${#Hsources[@]}; idx++ )); do
   if [ "${Hsources[idx]}" == "\${CMAKE_CURRENT_SOURCE_DIR}/"$NAME".h" ] ;
   then
     EXISTS=1
   fi
  done
  
  if [ $EXISTS == 0 ] ;
  then
    Hsources=( "${Hsources[@]}" $NewH )
    IFS=$'\n' 
    sorted=($(sort <<<"${Hsources[*]}"))
    unset IFS
    SET_AFTER_POS=-2
   
    for (( idx = 0; idx < ${#sorted[@]}; idx++ )); do
      if [ "${sorted[idx]}" == "\${CMAKE_CURRENT_SOURCE_DIR}/"$NAME".h" ] ;
      then
  	    SET_AFTER_POS=$(( idx-1 ))
  	    if [ $SET_AFTER_POS == -1 ] ; 
  	    then
  	  	  sed -i "\%${sorted[1]}%i \  \${CMAKE_CURRENT_SOURCE_DIR}/$NAME.h" "$CML_PATH"
  	    else
  	  	  sed -i "\%${sorted[$SET_AFTER_POS]}%a \  \${CMAKE_CURRENT_SOURCE_DIR}/$NAME.h" "$CML_PATH"
  	    fi
      fi	  	
    done
  
    # Add CPP source
    Hsources=( $(grep -e ".cpp" "${CML_PATH}") )
    NewCPP="\${CMAKE_CURRENT_SOURCE_DIR}/$NAME.cpp"
    Hsources=( "${Hsources[@]}" $NewCPP )
    IFS=$'\n' 
    sorted=($(sort <<<"${Hsources[*]}"))
    unset IFS
    SET_AFTER_POS=-2
   
    for (( idx = 0; idx < ${#sorted[@]}; idx++ )); do
      if [ "${sorted[idx]}" == "\${CMAKE_CURRENT_SOURCE_DIR}/"$NAME".cpp" ] ;
      then
  	    SET_AFTER_POS=$(( idx-1 ))
  	    if [ $SET_AFTER_POS == -1 ] ; 
  	    then
  	  	  sed -i "\%${sorted[1]}%i \  \${CMAKE_CURRENT_SOURCE_DIR}/$NAME.cpp" "$CML_PATH"
  	    else
  	  	  sed -i "\%${sorted[$SET_AFTER_POS]}%a \  \${CMAKE_CURRENT_SOURCE_DIR}/$NAME.cpp" "$CML_PATH"
  	    fi
      fi	  	
    done
    echo "$CML_PATH"
  else
    echo ""
  fi
}

createGtest()
{
  # ===== Create gtest file =====
  # Templates path
  TEMPLATES_PATH=${WOLF_SCRIPTS_PATH}/templates

  # Set the include and class names on the template files
  sed 's/header_file/'"${TYPE}s\/${NAME}.h"'/g' "${TEMPLATES_PATH}"/gtest_template.cpp > "${TEMPLATES_PATH}"/tmp.cpp
  sed 's/class_name/'"${CLASSNAME}"'/g' "${TEMPLATES_PATH}"/tmp.cpp > "${TEMPLATES_PATH}"/tmp2.cpp
  rm "${TEMPLATES_PATH}"/tmp.cpp

  FUNC_NAMES=
  # Insert dummy tests for all methods
  for (( idx = $((${#FuncList[@]}-1)); idx > -1; idx-- )); do
    TMP=$(echo ${FuncList[idx]} | sed -r 's/'"$D"'/ /g')
    TMP=$(echo $TMP | sed -r 's/'"virtual"'/ /g')
    TMP=$(echo $TMP | sed -r 's/'"=0"'/ /g')
    TMP=$(echo $TMP | sed -r 's/'" = 0"'/ /g')
    TMP=${TMP%$TMP##*[![:space:]]}
    TMP=$(echo $TMP | sed 's/(.*//g' | sed 's/.* //g')
    sed -i "/\[Class methods\]/a TEST_F($CLASSNAME, $TMP)\n\{\n\}\n" "${TEMPLATES_PATH}"/tmp2.cpp 
  done

  GTEST_PATH="${WOLF_ROOT}"/src/test/gtest_${NAME}.cpp
  mv "${TEMPLATES_PATH}"/tmp2.cpp $GTEST_PATH
}

updateCMakeListsGTest()
{
  CML_GTEST_PATH="${WOLF_ROOT}/src/test/CMakeLists.txt"

  DERIVED_TXT=$(cat $CML_GTEST_PATH  | sed '/^#/!d')
  DERIVED_TXT="${DERIVED_TXT##*\# ------- Now Derived classes ----------}"
  DERIVED_TXT="${DERIVED_TXT%%\# ------- Now Core classes Serialization ----------*}"
  DERIVED_TXT=$(echo "$DERIVED_TXT" | sed '/^\s*$/d')
  IFS=$'\n'
  DERIVED_TXT=($(echo "$DERIVED_TXT" | sed -r 's/'"#\s"'/ /g'))
  for (( idx = 0; idx < ${#DERIVED_TXT[@]}; idx++ )); do
    DERIVED_TXT[idx]=${DERIVED_TXT[idx]#" "}
    DERIVED_TXT[idx]=$(echo ${DERIVED_TXT[idx]} | sed 's/ .*//')
  done

  New="${CLASSNAME}"

  # Check if already exists
  EXISTS=0
  for (( idx = 0; idx < ${#DERIVED_TXT[@]}; idx++ )); do
   if [ "${DERIVED_TXT[idx]}" == "$New" ] ;
   then
     EXISTS=1
   fi
  done
  
  if [ $EXISTS == 0 ] ;
  then
    DERIVED_TXT=( "${DERIVED_TXT[@]}" $New )
    IFS=$'\n' 
    sorted=($(sort <<<"${DERIVED_TXT[*]}"))
    unset IFS
    SET_BEFORE_POS=-2
    for (( idx = 0; idx < ${#sorted[@]}; idx++ )); do
      if [ "${sorted[idx]}" == $New ] ;
      then
        SET_BEFORE_POS=$(( idx+1 ))
        if [ $SET_BEFORE_POS == ${#sorted[@]} ] ; 
        then
          SET_BEFORE_POS=$(( idx ))
        fi
      fi	  	
    done
    sed -i "\%${sorted[$SET_BEFORE_POS]}%i # $New test\nwolf_add_gtest(gtest_$NAME gtest_$NAME.cpp\ntarget_link_libraries(gtest_$NAME \$\{PROJECT_NAME\}\n" "${CML_GTEST_PATH}"
    echo "$CML_GTEST_PATH"
  else
    echo ""
  fi
}

# ============================
# ============================
