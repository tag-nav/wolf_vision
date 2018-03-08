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
if [ -z "$TYPE" ] && [ -z "$NAME" ] && [ -z "$BASE" ]
then
   showHelp	
   exit
fi
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
NAME=$(capitalizeDiminutives $NAME)
NAME_CAP=$(UpperCase $NAME)
NAME_CAP1=$(UpperCaseFirstLetter $NAME)

BASE=$(LowerCase $BASE)
BASE=$(capitalizeDiminutives $BASE)
BASE_CAP=$(UpperCase $BASE)
BASE_CAP1=$(UpperCaseFirstLetter $BASE)

TYPE=$(LowerCase $TYPE)
TYPE_CAP=$(UpperCase $TYPE)
TYPE_CAP1=$(UpperCaseFirstLetter $TYPE)

CLASSNAME="$TYPE_CAP1$BASE_CAP1$NAME_CAP1"
CLASSNAME=$(echo ${CLASSNAME} | sed 's/_\(.\)/\U\1/g') # remove "_" and capitalize next character

BASECLASSNAME=$(echo ${BASE} | sed 's/_\(.\)/\U\1/g') # remove "_" and capitalize next character
BASECLASSNAME=$TYPE_CAP1$(echo "$(echo "$BASECLASSNAME" | sed 's/.*/\u&/')") 

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
