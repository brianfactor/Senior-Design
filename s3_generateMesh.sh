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
echo "--s3_generateMesh.sh--" >> log.txt

#check for empty arguments
if [ -z $1 ] || [ -z $2 ]
  then
	echo "Invalid arguments." >> log.txt
	echo "USAGE: ./s3_generateMesh.sh inputpc.ply outputMesh.ply" >> log.txt
	echo "USAGE: ./s3_generateMesh.sh inputpc.ply outputMesh.ply"
	exit 1
fi
#check that file exists
while [ ! -e $1 ]
  do
	echo "Error: File $1 doesn't exist." >> log.txt
	sleep 5
	#exit 1
done

echo "Applying mesh to $1. output to $2" >> log.txt
# xvfb-run added to we can call it thru ssh (without x)
eval `xvfb-run --server-args="-screen 0, 1024x768x24" meshlabserver -i $1 -o $2 -s DelaunayTriangulation.mlx`

testResult $1 # check if it returned errors
exit 0 # else return with no errors
