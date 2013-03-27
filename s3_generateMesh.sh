#!/bin/bash
# Last step - run this after correlating clouds from all perspectives (after running step2 x12)
# USAGE: ./s3_generateMesh.sh inputpc.ply outputMesh.ply

# this handles errors based on the return value of each script we call here
function testResult {
	result=$?
	if [ $result != 0 ]
	  then
		echo "Error $result during meshing of $1" >> log.txt
		exit $result
	fi
}

echo "--Log record `date`--" >> log.txt

#check for empty arguments
if [ -z $1 ] || [ -z $2 ]
  then
	echo "Invalid arguments." >> log.txt
	echo "USAGE: ./s3_generateMesh.sh inputpc.ply outputMesh.ply" >> log.txt
	echo "USAGE: ./s3_generateMesh.sh inputpc.ply outputMesh.ply"
	exit 1
fi
#check that file exists
if [ ! -e $1 ]
  then
	echo "Error: File doesn't exist." >> log.txt
	exit 1
fi

echo "Applying mesh to $1. output to $2" >> log.txt
meshlabserver -i $1 -o $2 -s DelaunayTriangulation.mlx

testResult $1 # check if it returned errors
exit 0 # else return with no errors
