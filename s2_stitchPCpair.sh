#!/bin/bash
# run this after calling step1 twice and then once every time it's called again
# ie: run on a pair of pointclouds
# USAGE: ./s3_stichPCpair.sh pcout00.ply pcstitch.ply pcstitch.ply

# this handles errors based on the return value of each script we call here
function testResult {
	result=$?
	if [ $result != 0 ] 
	  then
		echo "Error $result during stitching of $1 and $2" >> log.txt
		exit $result
	fi
}

echo "--Log record `date`--" >> log.txt
echo "--s2_stitchPCpai.sh--" >> log.txt

#check for empty arguments
if [ -z $1 ] || [ -z $2 ] || [ -z $3 ]
  then
	echo "Invalid arguments." >> log.txt
	echo "USAGE: ./s3_stichPCpair.sh pcout00.ply pcstitch.ply pcstitch.ply" >> log.txt
	echo "USAGE: ./s3_stichPCpair.sh pcout00.ply pcstitch.ply pcstitch.ply"
	exit 1
fi
#check that files exist
while [ ! -e $1 ] || [ ! -e $2 ]
  do
	echo "File $1 or $2 doesn't exist." >> log.txt
	sleep 5
	#exit 1
done

#echo "Stitching clouds $1 and $2 together, storing output in $3" >> log.txt
#eval "./pairwise_incremental_registration $1 $2 -30 $3"

testResult $1 $2 	# check if it returned errors
exit 0				# else success
