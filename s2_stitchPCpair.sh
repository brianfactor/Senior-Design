#!/bin/bash
# run this after calling step1 twice and then once every time it's called again
# ie: run on a pair of pointclouds
# USAGE: ./s3_stichPCpair.sh pcout00.ply pcstitch.ply pcstitch.ply

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
if [ -z $1 ] || [ -z $2 ] || [ -z $3 ]
  then
	echo "USAGE: ./s3_stichPCpair.sh pcout00.ply pcstitch.ply pcstitch.ply"
	exit 1
fi

echo "Stitching clouds $1 and $2 together, storing output in $3"
#./pairwise_incremental_registration $1 $2
mv 1.ply $3
testResult # check if it returned errors
exit 0 # else success

