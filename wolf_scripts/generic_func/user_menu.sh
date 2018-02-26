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
if [ -z "$TYPE" ] || [ -z "$NAME" ] || [ -z "$BASE" ]
then
   showHelp	
   exit
fi

NAME=$(LowerCase $NAME)
NAME_CAP=$(UpperCase $NAME)
NAME_CAP1=$(UpperCaseFirstLetter $NAME)
BASE=$(LowerCase $BASE)
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
