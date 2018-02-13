#!/bin/bash

if [ ! -x UAV ]; then
	echo "no UAV or it is not executable"
	exit 1
fi

if [ $# -ne 1 ]; then
	echo "Usage: $0 caseFileDir"
	echo "Example: $0 ../cases/X2000Y2000N800K8"
	exit 1
fi

if [ ! -d $1 ]; then
	echo "$1 is not a valid dir"
	exit 1
fi

rm -f $1/statistic.csv

for casefile in $1/case_*.txt; do
	./UAV $casefile $1/statistic.csv > /dev/null
	if [ $? -ne 0 ]; then
		echo "error occurs when executing $casefile"
		exit 1
	fi
	postfix=${casefile#$1/case}
	prefix=${postfix%txt}
	mv UAVs.csv $1/UAVs${prefix}csv
	mv servedUsers.csv $1/servedUsers${prefix}csv
done

echo "statistic.csv is generated in dir $1"

