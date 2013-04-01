#!/bin/bash
# processes all steps necessary to make a 3d Model
# call with no arguments

# clear log
#rm log.txt

# run x 12
#i=1
#set i=3 now due to background filtering
i=3
j=0

#rectify background images
eval "./rectify 2.jpg 1.jpg bgl.jpg bgr.jpg"

while [ $i -lt 27 ]
  do
	let "j = i + 1"
	time ./s1_correlateImg.sh $i.jpg $j.jpg $i.ply >> processLog.txt
	
	# run x 11
	#if [ $i == 3 ]
	  #then
		#./s2_stitchPCpair.sh 1.ply 3.ply outstitch.ply >> processLog.txt
	#fi
	#if [ $i -gt 4 ]
	#  then
		#./s2_stitchPCpair.sh outstitch.ply $i.ply outstitch.ply >> processLog.txt
	#fi
	
	let "i = i + 2"
done

# run once
#./s3_generateMesh.sh outstitch.ply output.ply >> processLog.txt
time ./s3_generateMesh.sh 3.ply 5.ply 7.ply 9.ply 11.ply 13.ply 15.ply 17.ply 19.ply 21.ply 23.ply 25.ply .1 output.ply >> processLog.txt

#backup files
DIR=`date +%H%M%D | sed 's/\///g'`
mkdir $DIR
mv *.jpg *.txt *.ply *.png $DIR
cp $DIR/output.ply .
touch processLog.txt
