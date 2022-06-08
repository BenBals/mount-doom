#!/bin/bash
# plots the given input file using graphviz

if [ "$#" -ne 1 ]; then
	echo "Usage: $0 <dot-file>"
	exit 1
fi

echo "Visualizing $1"
dot -Ksfdp -Tsvg $1 -o${1%.dot}.svg
