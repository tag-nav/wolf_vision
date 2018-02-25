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
  
  echo $NAME
  
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

  #for (( idx = 0; idx < ${#FuncList[@]}; idx++ )); do
  for (( idx = $((${#FuncList[@]}-1)); idx > -1; idx-- )); do
  	TMP=$(echo ${FuncList[idx]} | sed -r 's/'"$D"'/ /g')
    TMP=$(echo $TMP | sed -r 's/'"virtual"'/ /g')
    TMP=$(echo $TMP | sed -r 's/'"=0"'/ /g')
    TMP=$(echo $TMP | sed -r 's/'" = 0"'/ /g')
    TXTH="${TMP%"$TMP##*[![:space:]]}"}"
    TXTCPP_3=$(echo $TXTH | sed 's/.*(//g')
    TXTCPP_2=$(echo $TXTH | sed 's/(.*//g' | sed 's/.* //g')
    TXTCPP_1=$(echo $TXTH | sed 's/'"$TXTCPP_2"'.*//g')
 
    # CPP file
    FUNCNAME=${TXT%*\(}
    sed -i "/\[base class inherited methods\]/a ${TXTCPP_1}${CLASSNAME}::${TXTCPP_2}(${TXTCPP_3}\n\{\n\}\n" "$NAME_CPP_PATH"  
  done    
    
  rm "${WOLF_SCRIPTS_PATH}"/class.h
}

# ============================
# ============================
