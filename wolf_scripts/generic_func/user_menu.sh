NAME=
BASE=
TYPE=
while getopts ":t:n:b:" OPTION
do
  case ${OPTION} in
    t)
       TYPE=${OPTARG}
       ;;
    n)
       NAME=${OPTARG}
       ;;
    b)
       BASE=${OPTARG}
       ;;
    *) 
       showHelp 
       exit
       ;;
  esac
done
if [ -z "$TYPE" ] ;
then
   echo ""
   echo "${RED}  [ERROR]: Missing TYPE value (-t).${NC}"
   echo ""
   showHelp	
   exit
fi

if [ -z "$NAME" ] ;
then
   echo ""
   echo "${RED}  [ERROR]: Missing NAME value (-n).${NC}"
   echo ""
   showHelp	
   exit
fi
if [ -z "$BASE" ] ;
then
   echo ""
   echo "${RED}  [ERROR]: Missing BASE value (-b).${NC}"
   echo ""
   showHelp	
   exit
fi

if ! [ "$TYPE" == "capture" ] && ! [ "$TYPE" == "constraint" ] && ! [ "$TYPE" == "feature" ] && ! [ "$TYPE" == "processor" ] && ! [ "$TYPE" == "sensor" ]
then
   echo ""
   echo "${RED} --> Incorrect type \"$TYPE\". Please check the following instructions: ${NC}"
   echo ""
   showHelp
   exit 1
fi

NAME=$(LowerCase $NAME)
if [[ $NAME = *"2d"* ]]; 
then
  NAME=$(echo "${NAME/2d/2D}")
elif [[ $NAME = *"3d"* ]];
then
  NAME=$(echo "${NAME/3d/3D}")
fi
NAME_CAP=$(UpperCase $NAME)
NAME_CAP1=$(UpperCaseFirstLetter $NAME)

BASE=$(LowerCase $BASE)
if [[ $BASE = *"2d"* ]]; 
then
  BASE=$(echo "${BASE/2d/2D}")
elif [[ $BASE = *"3d"* ]];
then
  BASE=$(echo "${BASE/3d/3D}")
fi
BASE_CAP=$(UpperCase $BASE)
BASE_CAP1=$(UpperCaseFirstLetter $BASE)

TYPE=$(LowerCase $TYPE)
TYPE_CAP=$(UpperCase $TYPE)
TYPE_CAP1=$(UpperCaseFirstLetter $TYPE)

CLASSNAME="$TYPE_CAP1$BASE_CAP1$NAME_CAP1"

if ! [[ $NAME =~ $TYPE ]] ;
then
  if [[ $BASE =~ .*base*. ]] ;
  then
    NAME="$TYPE"_"$NAME";  
  else
    NAME="$TYPE"_"$BASE"_"$NAME";
  fi
fi
if ! [[ $BASE =~ $TYPE ]] ;
then
    BASE="$TYPE"_"$BASE";
fi

# Useful derivatives
BASECLASSNAME=$(echo ${BASE##*_})
BASECLASSNAME=$TYPE_CAP1$(echo "$(echo "$BASECLASSNAME" | sed 's/.*/\u&/')") 
