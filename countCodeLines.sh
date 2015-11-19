#!/bin/bash
cloc --exclude-list-file=countCodeExclude.txt  src/ tests/ tools/

count=`cloc --exclude-list-file=countCodeExclude.txt  src/ tests/ tools/ | awk '{if ($1=="SUM:") print $5}'`
echo
echo "TOTAL Line Count: $count"

echo "`date` $count" >> codeLineCountHistory.dat
