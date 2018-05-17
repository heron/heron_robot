#!/bin/bash
while read p; do
	if [[ $p == x* ]] || [[ $p == y* ]] || [[ $p == z* ]]
		then
		num=${p:3}
		axis=${p:0:1}
		echo "<rosparam param=\"/mag_bias/${axis}\">${num}</rosparam>"
	fi		
done
