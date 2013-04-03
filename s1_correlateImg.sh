#!/bin/bash
# run this on two images: left ($1) and right ($2)
# output is a .ply pointcloud ($3)
# USAGE: ./s1_correlateImg left00.jpg right00.jpg out00.ply

# this handles errors based on the return value of each script we call here
function testResult {
	result=$?
	if [ $result != 0 ] 
	  then
		echo "Error $result during correlation of $1 and $2" >> log.txt
		exit $result
	fi
}

echo "--Log record `date`--" >> log.txt
echo "--s1_correlateImg.sh--" >> log.txt

#check for empty arguments
if [ -z $1 ] || [ -z $2 ] || [ -z $3 ]
  then
	echo "Invalid arguments." >> log.txt
	echo "USAGE: ./s1_correlateImg left00.jpg right00.jpg out00.ply" >> log.txt
	echo "USAGE: ./s1_correlateImg left00.jpg right00.jpg out00.ply"
	exit 1
fi
#check that files exist
while [ ! -e $1 ] || [ ! -e $2 ]
  do
	echo "File $1 or $2 doesn't exist." >> log.txt
	#exit 1
	sleep 5
done

echo "Rectifying images $1 and $2" >> log.txt
#echo "rectify $1 $2 leftrectified.jpg rightrectified.jpg"
eval "./rectify $2 $1 leftrectified.jpg rightrectified.jpg"
testResult $1 $2

while [ ! -e leftrectified.jpg ] || [ ! -e rightrectified.jpg ]
  do
	echo "File leftrectified.jpg or rightrectified.jpg doesn't exist." >> log.txt
	sleep 5
	#exit 1
done

echo "Correlating $1 and $2 and filtering filtering point cloud $3" >> log.txt
#echo "python stereo_match.py leftrectified.jpg rightrectified.jpg $3"
python ./stereo_match.py leftrectified.jpg rightrectified.jpg $3
testResult $1 $2

#./statistical_removal $3 500 .5 # don't use - Drew's is better

testResult
# successful exit
exit 0
