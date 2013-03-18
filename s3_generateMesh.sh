#!/bin/bash
# Last step - run this after correlating clouds from all perspectives (after running step2 x12)
# USAGE: ./s3_generateMesh.sh inputpc.ply outputMesh.ply

# this handles errors based on the return value of each script we call here
function testResult {
	result=$?
	if [ $result != 0 ] 
	  then
		echo "Error $result during meshing of $1"
		exit $result
	fi
}

#check for empty arguments
if [ -z $1 ] || [ -z $2 ]
  then
	echo "USAGE: ./s3_generateMesh.sh inputpc.ply outputMesh.ply"
	exit 1
fi

echo "Applying mesh to $1. output to $2"
meshlabserver -i $1 -o $2 -s DelaunayTriangulation.mlx
testResult # check if it returned errors
exit 0 # else return with no errors

