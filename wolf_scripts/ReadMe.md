# SCRIPTS

Helpful scripts to create WOLF tree elements (e.g. processors)

## Installation and usage

  * Move to the scripts folder   
  `cd $WOLF_ROOT/wolf_scripts`  

  * Run the following line to set your `bashrc`. This will allow you to execute the scripts from any console path.  
  ``echo "source $WOLF_ROOT/wolf_scripts/setup.bash" >> ~/.bashrc``

## Main scripts

#### Create a new processor

Creates the main structure of a new processor, including .cpp and .h files, doxygen files and a gtest for that processor in `test/processors/`. 

  * Example of usage  
  `create_wolf_processor.sh -n example -b tracker` 

  * Options:
    - n: Processor name
    - b: Processor base class (inherited from this class)
    - h: Help


