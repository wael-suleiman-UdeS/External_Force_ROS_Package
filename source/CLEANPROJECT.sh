
#######################################################################
# 			Project cleaning script
#
# Created by : Louis Hawley
# Date: 19/01/2016
# Modified by Remy Rahem
# Date: 04/05/2018
#
# Description : This script cleans the project by removing the build/ 
# and devel/ folder. Also removes build logs and temporary files.
#
# Usage : Execute the following command in a terminal :
#	. CLEANPROJECT.sh
#
#######################################################################
time(
    rm -r build/ devel/ 2> /dev/null
    rm ../Doc/packages.txt 2> /dev/null
    rm buildErrors.log 2> /dev/null
    rm *~ 2> /dev/null
    echo 
    echo Time spent to clean the project:
)
echo
