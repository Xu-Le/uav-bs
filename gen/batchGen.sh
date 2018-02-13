#!/bin/bash

if [ ! -x genCases ]; then
	echo "no genCases or it is not executable"
	exit 1
fi

if [ $# -ne 4 ]; then
	echo "Usage: $0 X Y N K"
	exit 1
fi

CASE_DIR=../cases/X$1Y$2N$3K$4
rm -rf $CASE_DIR
echo "create dir $CASE_DIR"

mkdir $CASE_DIR
for seed in $(seq 1 100); do
	./genCases -x $1 -y $2 -u $3 -k $4 -s $seed case_$seed.txt
	mv case_$seed.txt $CASE_DIR/case_$seed.txt
	mv user_$seed.csv $CASE_DIR/user_$seed.csv
done

echo "cases are generated"

