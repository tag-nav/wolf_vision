#! /bin/bash

# ===== FUNCTIONS =====

# Bash colors
RED=$(tput setaf 1)
GREEN=$(tput setaf 2)
YELLOW=$(tput setaf 3)
BLUE=$(tput setaf 4)
CYAN=$(tput setaf 6)
NC=$(tput sgr 0) # No Color

askIfGitBranch()
{
 read -p "${CYAN}Do you want to create a new git branch (y/n)?  ${NC}" ANSWER
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
       echo "${RED}  [ERROR]: Incorrect answer.${NC} Please type y or n"
       exit
       ;;
  esac
done   

if [ "$DOIT" == "y" ];
then
  if [ -z "$1" ]
  then
    echo "${RED}  [ERROR]: No argument supplied.${NC}"
  fi
  cd $WOLF_ROOT 
  git checkout -b $1
  echo "${GREEN} \--> Created git branch $1${NC}"
fi
echo ""    
}

getEnvVariable()
{
  local var=`echo "${!1}"`
  if [ -z "${var}" ]
  then
    echo "${RED}  [ERROR]: The environment variable ${1} has not been defined.${NC} Please see the wiki documentation for instructions on how to create it." >&2
    exit
  fi
  echo $var
}

showHelp() 
{
  echo "******************************************"  
  echo "${BLUE}Script to generate a WOLF $1 class${NC}"
  echo "******************************************"
  echo " Required parameters:"
  echo "  -${CYAN}t${NC}: type. Any of the following [ ${CYAN}capture${NC} | ${CYAN}constraint${NC} | ${CYAN}feature${NC} | ${CYAN}processor${NC} | ${CYAN}sensor${NC}]"
  echo "  -${CYAN}n${NC}: name"
  echo "  -${CYAN}b${NC}: base class (inheritance)"
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
  		echo "${RED}  [ERROR]: No $1 provided, aborting ...${NC}"
  		exit
	fi
}

UpperCase()
{
	if [ $1 ]
	then
  		echo $1 | tr '[:lower:]' '[:upper:]'
	else
  		echo "${RED}  [ERROR]: No $1 provided, aborting ...${NC}"
  		exit
	fi
}

UpperCaseFirstLetter()
{
  echo "$(echo "$1" | sed 's/.*/\u&/')"
}

capitalizeDiminutives()
{
  RES=$1
  if [[ $1 =~ .*"2d"*. ]] ;  
  then
    RES=$(echo "${1/2d/2D}")
  fi
  if [[ $1 =~ .*"3d"*. ]] ;
  then
    RES=$(echo "${RES/3d/3D}")
  fi
  if [[ $1 =~ .*"gps"*. ]] ;
  then
    RES=$(echo "${RES/gps/GPS}")
  fi  
  if [[ $1 =~ .*"imu"*. ]] ;
  then
    RES=$(echo "${RES/imu/IMU}")    
  fi
  echo $RES
}

getFilePath()
{
  if find $WOLF_ROOT -name "${1}" -print -quit | grep -q '^'; 
  then
    echo "$(find "$WOLF_ROOT/src" -name ${1} )"
  else
    echo ""
  fi
}

fillWithBaseConstructorParameters()
{
  # Header: class constructor with base class parameters	
  H_TXT=$(cat $BASE_H_PATH)
  H_TXT="${H_TXT#*class $BASECLASSNAME :}"
  BASECLASS_TXT="${H_TXT%%\n\};*}"
  PARAMS="${BASECLASS_TXT#*$BASECLASSNAME(}"

  BASE_DERIVES_FROM_AUTODIFF=""
  if [[ $PARAMS =~ .*ConstraintAutodiff*. ]] && ! [[ $BASECLASSNAME =~ .*ConstraintAutodiff*. ]] ;
  then
    BASE_DERIVES_FROM_AUTODIFF="TRUE"
  fi    

  PARAMS="${PARAMS%%\) :*}" # in case of inheritance
  PARAMS="${PARAMS%%\);*}"

  if [[ "$PARAMS" == *"const std::string& _type"* ]] ;
  then
  	PARAMS="${PARAMS#*_type,}"
   	NAME_STR="${NAME#${TYPE}_}"
   	NAME_STR=$(UpperCase $NAME_STR)
  fi

  OLD=" class_name();"
  NEW="\ \ \ \ \ \ \ \ ${CLASSNAME}(${PARAMS});"
  NEW=${NEW//$'\n'/} # Remove all newlines.
  NEW=${NEW%$'\n'}   # Remove a trailing newline.
  NEW=$(echo $NEW | sed 's/^ *//g' | sed 's/ *$//g') # remove extra whitespaces
  sed '/'"${OLD}"'/c'"${NEW}"'' "${TEMPLATES_PATH}"/class_template.h > "${TEMPLATES_PATH}"/tmp.h

  # CPP: class constructor with base class parameters	
  PARAMS="$PARAMS," # add , at the end to ease things
  PARAMS=$(echo "$PARAMS" | sed 's/\ =.*,\ /,\ /g')
  PARAMS=$(echo "$PARAMS" | sed 's/\ =.*,/,/g')    
  PARAMS="${PARAMS::-1}" # remove , from the end

  OLD="class_name::class_name() :"
  NEW="class_name::class_name(${PARAMS}) :"
  NEW=${NEW//$'\n'/} # Remove all newlines.
  NEW=${NEW%$'\n'}   # Remove a trailing newline.
  NEW=$(echo $NEW | sed 's/^ *//g' | sed 's/ *$//g') # remove extra whitespaces
  sed '/'"${OLD}"'/c'"${NEW}"'' "${TEMPLATES_PATH}"/class_template.cpp > "${TEMPLATES_PATH}"/tmp.cpp           

  # get only the variable names
  PARAMS_CLEAN=$(echo $PARAMS | sed -r 's/'","'/ /g')
  ar=($PARAMS_CLEAN)
  PARAMS_OBJ=
  for el in "${ar[@]}"; do
    if [[ $el == _* ]] 
    then
      PARAMS_OBJ+=", ${el}"
    fi
  done
  PARAMS_OBJ=${PARAMS_OBJ#","}
  if ! [[ -z $NAME_STR ]] ;
  then
    PARAMS_OBJ="\"$NAME_STR\", $PARAMS_OBJ"
  fi
  OLD="\        base_class()"
  NEW="\        base_class(${PARAMS_OBJ} )"
  sed '/'"${OLD}"'/c'"${NEW}"'' "${TEMPLATES_PATH}"/tmp.cpp > "${TEMPLATES_PATH}"/tmp2.cpp 
  rm "${TEMPLATES_PATH}"/tmp.cpp
  mv "${TEMPLATES_PATH}"/tmp2.cpp "${TEMPLATES_PATH}"/tmp.cpp
}

createHCPPFromTemplates()
{
  # Templates path
  TEMPLATES_PATH=${WOLF_SCRIPTS_PATH}/templates

  # ===== Create HEADER and CPP files =====

  # Pick initialization parameters from base class
  fillWithBaseConstructorParameters

  # CPP only for non-autodiff
  if ! [[ $BASECLASSNAME =~ .*ConstraintAutodiff*. ]] ;
  then
    #Set the TYPE and class names on the template files
    sed 's/header_file/'"${NAME}.h"'/g' "${TEMPLATES_PATH}"/tmp.cpp > "${TEMPLATES_PATH}"/tmp2.cpp
    sed 's/class_name/'"${CLASSNAME}"'/g' "${TEMPLATES_PATH}"/tmp2.cpp > "${TEMPLATES_PATH}"/tmp3.cpp
    sed 's/base_class/'"${BASECLASSNAME}"'/g' "${TEMPLATES_PATH}"/tmp3.cpp > "${TEMPLATES_PATH}"/tmp4.cpp
    rm "${TEMPLATES_PATH}"/tmp.cpp
    rm "${TEMPLATES_PATH}"/tmp2.cpp
    rm "${TEMPLATES_PATH}"/tmp3.cpp
    # Rename and move files
    NAME_CPP_PATH="$WOLF_ROOT"/src/"$TYPE"s/"$NAME".cpp
    mv "${TEMPLATES_PATH}"/tmp4.cpp "$NAME_CPP_PATH"
  fi
  
  sed 's/base_header_file/'"${BASE}.h"'/g' "${TEMPLATES_PATH}"/tmp.h > "${TEMPLATES_PATH}"/tmp2.h
  sed 's/name_cap/'"${TYPE_CAP}_${BASE_CAP}_${NAME_CAP}"'/g' "${TEMPLATES_PATH}"/tmp2.h > "${TEMPLATES_PATH}"/tmp3.h
  sed 's/class_name/'"${CLASSNAME}"'/g' "${TEMPLATES_PATH}"/tmp3.h > "${TEMPLATES_PATH}"/tmp4.h
  sed 's/base_class/'"${BASECLASSNAME}"'/g' "${TEMPLATES_PATH}"/tmp4.h > "${TEMPLATES_PATH}"/tmp5.h
  rm "${TEMPLATES_PATH}"/tmp.h
  rm "${TEMPLATES_PATH}"/tmp2.h
  rm "${TEMPLATES_PATH}"/tmp3.h
  rm "${TEMPLATES_PATH}"/tmp4.h
  
  # Rename and move files
  NAME_H_PATH="$WOLF_ROOT"/src/"$TYPE"s/"$NAME".h
  mv "${TEMPLATES_PATH}"/tmp5.h "$NAME_H_PATH"
}

addAutodiffSpecifics()
{
  # Number of parameters
  echo ""
  echo "${CYAN}To create the class $CLASSNAME, you have to provide some info:${NC}"
  echo "" 

  PROMPT="${CYAN}- What is the size (dimensions) of the residual?${NC} (1 integer, followed by [ENTER]):"
  read -p "${PROMPT}" RESIDUAL_DIM; 
  if ! [[ "$RESIDUAL_DIM" =~ ^[0-9]+$ ]] ;
  then
    echo "${RED}  [ERROR]: Invalid input.${NC} Expecting a numeric value. Aborting..."
    exit 1.
  fi
  echo -en "\033[1A\033[2K"
  echo -en "\033[1A\033[2K"
  echo "${GREEN} \--> Setting residual size to $RESIDUAL_DIM.${NC}"
  echo ""
  PROMPT="${CYAN}- How many state blocks are going to be considered in the constraint?${NC} (1 integer, followed by [ENTER]):"
  read -p "${PROMPT}" NUM_STATES; 
  if ! [[ "$NUM_STATES" =~ ^[0-9]+$ ]] ;
  then
    echo "${RED}  [ERROR]: Invalid input.${NC} Expecting a numeric value. Aborting..."
    exit 1.
  fi
  echo -en "\033[1A\033[2K"
  echo -en "\033[1A\033[2K"
  echo "${GREEN} \--> Setting $NUM_STATES state blocks.${NC}"
  echo ""
  
  for (( idx = 0; idx < $NUM_STATES; idx++ )); do
     PROMPT="${CYAN}- Name of state $((idx+1))?${NC} (1 string, followed by [ENTER]):"
     read -p "${PROMPT}" STATENAME;
     PROMPT="${CYAN}- Size (dimensions) of state $((idx+1)) (${STATENAME})?${NC} (1 integer, followed by [ENTER]:"
     read -p "${PROMPT}" STATESIZE; 
     if ! [[ "$STATESIZE" =~ ^[0-9]+$ ]] ;
     then
       echo "${RED}  [ERROR]: Invalid input.${NC} Expecting a numeric value. Aborting..."
       exit 1.
     fi
     echo -en "\033[1A\033[2K"
     echo -en "\033[1A\033[2K"
     NAMES+=( "${STATENAME}" )
     SIZES+=( "${STATESIZE}" )
  done

  echo -en "\033[1A\033[2K"
  echo -n "${GREEN} \--> Setting state blocks: ${NC}"
  PARAMS=
  for (( idx = 0; idx < ${#NAMES[@]}; idx++ )); do
    echo -n "${GREEN}${NAMES[$idx]}(${SIZES[$idx]}) ${NC}"
    PARAMS+=( "const T* const _${NAMES[idx]},")
    PARAM_NUMS+=( "${SIZES[idx]},")
  done
  echo ""
  
  PARAMS[-1]=${PARAMS[-1]%?}
  PARAM_NUMS[-1]=${PARAM_NUMS[-1]%?}

  sed -i "s/public $BASECLASSNAME/public $BASECLASSNAME<$CLASSNAME, $RESIDUAL_DIM, ${PARAM_NUMS[*]}>/g" "$NAME_H_PATH"
  sed -i "/virtual \~$CLASSNAME/a\ \n\        \/\*\* \brief : compute the residual from the state blocks being iterated by the solver.\n \        \*\*\/\n\        template<typename T>\n\        bool operator ()(${PARAMS[*]}, const T* const _residuals) const;\n" "$NAME_H_PATH"
  sed -i "/\} \/\/ namespace wolf/a\ \n\/\/ Include here all headers for this class\n\/\/\#include <YOUR_HEADERS.h>\n\nnamespace wolf\n\{\n\ntemplate<typename T> bool $CLASSNAME::operator ()(${PARAMS[*]}, const T* const _residuals) const\n\{\n  std::cout << \"\\033[1;33m [WARN]:\\033[0m ${CLASSNAME}::operator () is empty.\" << std::endl;\n  \/\/ TODO: Implement\n  return true;\n\}\n\n\} \/\/ namespace wolf" "$NAME_H_PATH"
}

fillWithBaseVirtualMethods()
{
  if ! [ -z $BASE_DERIVES_FROM_AUTODIFF ] ;
  then
    echo "${YELLOW} [ WARN]: Base class $BASECLASSNAME derives from AUTODIFF template. New .h and .cpp files are left without inherited functions."
  else
  
    if [[ $BASECLASSNAME =~ .*ConstraintAutodiff*. ]] ;
    then
      addAutodiffSpecifics
    else
      # Get base class	
      H_TXT=$(cat $BASE_H_PATH)
      H_TXT="${H_TXT##*class $BASECLASSNAME :}"
      BASECLASS_TXT="${H_TXT%%\n\};*}"

      echo "class $BASECLASSNAME :$BASECLASS_TXT };" > "${WOLF_SCRIPTS_PATH}"/class.h

      # H file Get Virtual function declarations with help
      sed -e '/./{H;$!d;}' -e 'x;/ = 0;/!d;' "${WOLF_SCRIPTS_PATH}"/class.h > "${WOLF_SCRIPTS_PATH}"/tmp.h
      sed -r 's/'" = 0"'//g' "${WOLF_SCRIPTS_PATH}"/tmp.h > "${WOLF_SCRIPTS_PATH}"/tmp2.h
      sed -i -e "/virtual \~$CLASSNAME/r ${WOLF_SCRIPTS_PATH}/tmp2.h" "$NAME_H_PATH" 
      rm "${WOLF_SCRIPTS_PATH}"/tmp.h	
      rm ${WOLF_SCRIPTS_PATH}/tmp2.h
    
      # CPP file  
      FuncInBase=$(grep -e " = 0;" -e "=0;" "${WOLF_SCRIPTS_PATH}/class.h")
      rm "${WOLF_SCRIPTS_PATH}"/class.h
    
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
        TXTCPP_3=$(echo "$TXTCPP_3" | sed -r 's/\*\*+/XXXX/g') ## remove **
        TXTCPP_3=$(echo "$TXTCPP_3" | sed -r 's/\*+/YYYY/g') ## remove *
        sed -i "/\} \/\/ namespace wolf/i ${TXTCPP_1}${CLASSNAME}::${TXTCPP_2}(${TXTCPP_3}" "$NAME_CPP_PATH"
        if ! [[ $TXTCPP_1 =~ .*void*. ]]
        then
          sed -i "/${CLASSNAME}::${TXTCPP_2}(${TXTCPP_3}/a \{\n  std::cout << \"\\033[1;33m [WARN]:\\033[0m ${CLASSNAME}::${TXTCPP_2} is empty.\" << std::endl;\n  ${TXTCPP_1}return_var\{\}; \/\/TODO: fill this variable\n  return return_var;\n\}\n" "$NAME_CPP_PATH"
        else 
          sed -i "/${CLASSNAME}::${TXTCPP_2}(${TXTCPP_3}/a \{\n  std::cout << \"\\033[1;33m [WARN]:\\033[0m ${CLASSNAME}::${TXTCPP_2} is empty.\" << std::endl;\n\}\n" "$NAME_CPP_PATH"
        fi
        sed -i -r 's/'"XXXX"'/'"**"'/g' "$NAME_CPP_PATH" # add again **
        sed -i -r 's/'"YYYY"'/'"*"'/g' "$NAME_CPP_PATH" # add again *
      done    
    fi
  fi  
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
  	      if [ ${#sorted[@]} == 1 ] ;
  	      then 
  	        sed -i "\%HDRS_CONSTRAINT%a \  \${CMAKE_CURRENT_SOURCE_DIR}/$NAME.h" "$CML_PATH"
  	      else
    	    sed -i "\%${sorted[1]}%i \  \${CMAKE_CURRENT_SOURCE_DIR}/$NAME.h" "$CML_PATH"
    	  fi
  	    else
  	  	  sed -i "\%${sorted[$SET_AFTER_POS]}%a \  \${CMAKE_CURRENT_SOURCE_DIR}/$NAME.h" "$CML_PATH"
  	    fi
      fi	  	
    done
  
    if ! [[ $BASECLASSNAME =~ .*ConstraintAutodiff*. ]] ;
    then
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
  	        if [ ${#sorted[@]} == 1 ] ;
  	        then 
  	          sed -i "\%SRCS_CONSTRAINT%a \  \${CMAKE_CURRENT_SOURCE_DIR}/$NAME.cpp" "$CML_PATH"
  	        else
    	      sed -i "\%${sorted[1]}%i \  \${CMAKE_CURRENT_SOURCE_DIR}/$NAME.cpp" "$CML_PATH"
    	    fi
  	      else
  	  	    sed -i "\%${sorted[$SET_AFTER_POS]}%a \  \${CMAKE_CURRENT_SOURCE_DIR}/$NAME.cpp" "$CML_PATH"
  	      fi
        fi	  	
      done
    fi  
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
    if ! grep -q "$TMP" "${TEMPLATES_PATH}"/tmp2.cpp
    then
      sed -i "/\[Class methods\]/a TEST($CLASSNAME, $TMP)\n\{\n  std::cout << \"\\033[1;33m [WARN]:\\033[0m gtest for ${CLASSNAME} ${TMP} is empty.\" << std::endl;\n\}\n" "${TEMPLATES_PATH}"/tmp2.cpp
    fi
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
      if [ "${sorted[idx]}" == "$New" ] ;
      then
        SET_BEFORE_POS=$(( idx+1 ))
        if [ $SET_BEFORE_POS == ${#sorted[@]} ] ; 
        then
          SET_BEFORE_POS=$(( idx ))
        fi
      fi	  	
    done
    sed -i "\%${sorted[$SET_BEFORE_POS]}%i # $New test\nwolf_add_gtest(gtest_$NAME gtest_$NAME.cpp)\ntarget_link_libraries(gtest_$NAME \$\{PROJECT_NAME\})\n" "${CML_GTEST_PATH}"
    echo "$CML_GTEST_PATH"
  else
    echo ""
  fi
}

# ============================
# ============================
