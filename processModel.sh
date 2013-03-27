#!/bin/bash
# processes all steps necessary to make a 3d Model
# call with no arguments

# run x 12
i=1
j=0
while [ $i -lt 25 ]
  do
	let "j = i + 1"
	./s1_correlateImg.sh $i.jpg $j.jpg $i.ply
	
	# run x 11
	if [ $i == 3 ]
	  then
		./s2_stitchPCpair.sh 1.ply 3.ply outstitch.ply
	fi
	if [ $i -gt 4 ]
	  then
		./s2_stitchPCpair.sh outstitch.ply $i.ply outstitch.ply
	fi
	
	let "i = i + 2"
done

# run once
./s3_generateMesh.sh outstitch.ply output.ply
