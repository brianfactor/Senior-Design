#!/bin/bash
# run this on two images: left ($1) and right ($2)
# output is a .ply pointcloud ($3)
# USAGE: ./s1_correlateImg left00.jpg right00.jpg out00.ply

# this handles errors based on the return value of each script we call here
function testResult {
	result=$?
	if [ $result != 0 ] 
	  then
		echo "Error $result during correlation of $1 and $2"
		exit $result
	fi
}

#check for empty arguments
if [ -z $1 ] || [ -z $2 ] || [ -z $3 ]
  then
	echo "USAGE: ./s1_correlateImg left00.jpg right00.jpg out00.ply"
	exit 1
fi

echo "Correlating $1 and $2"
# python ./stereo_match $1 $2
# ./OpenCVReprojectImageToPointCloud $1 disp.ppm Q.xml
# need to control output to $3 somehow...
testResult

echo "Filtering point cloud $3"
#./statistical_removal $3 500 .5
# mv 1.x 2.x # intentionally fail
testResult

# successful exit - need to modify scripts so we can check their exit status
exit 0

