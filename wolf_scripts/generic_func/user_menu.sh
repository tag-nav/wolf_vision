NAME=
BASE=
TYPE=

while getopts ":t:n:b:" OPTION
do
  case $OPTION in
    t)
       TYPE=$OPTARG
       ;;
    n)
       NAME=$OPTARG
       ;;
    b)
       BASE=$OPTARG
       ;;
    ?)
       showHelp 
       exit
       ;;
  esac
done

NAME=$(LowerCase $NAME)
NAME_CAP=$(UpperCase $NAME)
BASE=$(LowerCase $BASE)
BASE_CAP=$(UpperCase $BASE)
TYPE=$(LowerCase $TYPE)
TYPE_CAP=$(UpperCase $TYPE)

# Check naming (add type in front to keep WOLF naming format)
if ! echo "$NAME" | grep -q "$TYPE_" ;  
then
    NAME=$TYPE_$NAME;
fi
if ! echo "$BASE" | grep -q "$TYPE_" ; 
then
    BASE=$TYPE_$BASE;
fi