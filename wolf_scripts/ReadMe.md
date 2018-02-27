# SCRIPTS

Helpful scripts to create WOLF tree elements (e.g. processors)

## Installation

#### Dependencies

  * __RealPath__ (Required): Install with the following command.
  
  ..*`sudo apt-get install realpath`
  
#### SCRIPTS installation  

  * Move to the scripts folder
  
  ..*`cd $WOLF_ROOT/wolf_scripts`  

  * Run the following line to set your `bashrc`. This will allow you to execute the scripts from any console path.
  
  ..*`echo "source $WOLF_ROOT/wolf_scripts/setup.bash" >> ~/.bashrc`

## Usage

#### Create a new derived class

Creates the main structure including .cpp and .h files, and a gtest for the new derived class (all in corresponding folders of WOLF directory. 

  * Example of usage  
  `create_wolf_processor.sh -t processor -n example -b tracker` 

  * Options:
    - t: Type (e.g. sensor, processor, constraint, ...)
    - n: Name 
    - b: Base class (inherited from this class)
    - h: Help


