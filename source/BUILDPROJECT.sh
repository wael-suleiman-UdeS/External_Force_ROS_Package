#######################################################################
# 			Project Building script
#
# Created by : Louis Hawley
# Date: 19/01/2016
# Modified by Remy Rahem
# Date: 04/05/2018
#
# Description : This script builds the project and source the 
# executable so that they can be called via roslaunch and rosrun 
# command
#
# Usage : 	Execute the following command in a terminal :
#				. BUILDPROJECT.sh
#			or the following, if a cleaning is required:
#				. BUILDPROJECT.sh 1
#
#######################################################################
ARG1=${1:-0}
time(if [ "$ARG1" -eq "1" ]
then
	rm -r build/ devel/ 2> /dev/null
	rm ../Doc/packages.txt 2> /dev/null
	rm buildErrors.log 2> /dev/null
fi
catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles" 2> >(tee buildErrors.Log)
source devel/setup.bash
rospack list | grep $(pwd) > ../Doc/packages.txt
echo
echo Build completed, a package list can be found at $(cd ../ && pwd)/Doc/packages.txt
echo Time spent to build the project:
)
echo
