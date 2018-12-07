# SCRIPTS for DEVELOPERS

Helpful scripts to create WOLF tree elements (e.g. processors)

## Installation

#### Dependencies

  * __RealPath__ (Required): Install with the following command.
  
    ```
    sudo apt-get install realpath
    ```
  
  * __RealPath__ (Required, MacOSX): The realpath package above is not available for MacOSX. 
    Here is an out-of-the-box alternative 
    (credits to WaffleSouffle: https://stackoverflow.com/questions/3572030/bash-script-absolute-path-with-osx):
  
    - Create a small project `realpath` (here in `$HOME/dev/` as an example)

    ```
    cd $HOME/dev
    mkdir realpath
    cd realpath
    ```
    
    - Create a file `realpath.c` with the contents:
  
    ```c
    // realpath.c
	#include <stdio.h>
	#include <stdlib.h>

	int main (int argc, char* argv[])
	{
  		if (argc > 1) {
    		for (int argIter = 1; argIter < argc; ++argIter) {
      			char *resolved_path_buffer = NULL;
      			char *result = realpath(argv[argIter], resolved_path_buffer);

      			puts(result);

      			if (result != NULL) {
        				free(result);
      			}
    		}
  		}

  	return 0;
	}
	```
	
	- Create a file `Makefile` with the contents (leading whitespace are TABs!!)
	
	```
	#Makefile
	OBJ = realpath.o

	%.o: %.c
		$(CC) -c -o $@ $< $(CFLAGS)

	realpath: $(OBJ)
		gcc -o $@ $^ $(CFLAGS)
	```
	
	- Then compile with `make`:
	
    ```
    make
    ```
	 
	- and put in a soft link with:
	
    ```
    ln -s $(pwd)/realpath /usr/local/bin/realpath
    ```
  
#### SCRIPTS installation  

  * Move to the scripts folder
  
    ```
    cd $WOLF_ROOT/wolf_scripts
    ```

  * Run the following line to set your `bashrc`. This will allow you to execute the scripts from any console path.
  
    ```
    echo "source $WOLF_ROOT/wolf_scripts/setup.bash" >> ~/.bashrc
    ```

## Usage

#### Create a new derived class

Creates the main structure including .cpp and .h files, and a gtest for the new derived class (all in corresponding folders of WOLF directory. 

  * Example of usage  
    ```
    wolf_create.sh -t processor -n example -b tracker
    ```
    
    will create a new class `ProcessorTrackerExample`, with files:
    
    ```
    src/processors/processor_tracker_example.h
    src/processors/processor_tracker_example.cpp
    src/test/gtest_processor_tracker_example.cpp
    ```
      
    and modify the `CMakeLists.txt` files in the directories `src/processors/` and `src/test/`.

  * Options:
    - t: Type. Any of the following "[ capture | constraint | feature | processor | sensor ]"
    - n: Name 
    - b: Base class (inherited from this class)