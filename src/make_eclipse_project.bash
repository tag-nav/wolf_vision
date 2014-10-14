# Usage: cd to wolf/, then > source src/make_eclipse_project.bash
#
# If you are not in wolf/ then the nothing is done. This protects your directories.

cd ./build && cmake -DCMAKE_CXX_COMPILER=g++ -G"Eclipse CDT4 - Unix Makefiles" .. && cd ..


