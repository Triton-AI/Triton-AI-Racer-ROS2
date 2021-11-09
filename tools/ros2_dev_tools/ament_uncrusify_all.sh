#! /bin/bash

if  [[ -d "$1" ]]; then
    find "$1" -name "*.cpp" -exec ament_uncrustify {} --reformat \;
    find "$1" -name "*.hpp" -exec ament_uncrustify {} --reformat \;
else
   echo "Usage: ./ament_uncrustify_all.sh path/to/target/directory/"
fi