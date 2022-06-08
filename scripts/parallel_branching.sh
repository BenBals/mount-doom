#!/usr/bin/env bash
cpus=$(grep -c ^processor /proc/cpuinfo)

echo "Execute this script from within the build folder. doom should be in ./src"

if command -v xargs &>/dev/null; then
	instance_names=$(printf "%s\n" ../instances/exact_public/* | xargs -L 1 basename)
	echo "$instance_names" | xargs -L1 -P $cpus -I{} sh -c "timeout 1m ./src/doom --branching --matching {} > ../solutions/{}"
else
	echo "to better utilize your resources install xargs\n"

	for file in ../instances/exact_public/*; do
	  echo "Starting $file"
	  nice -n 1 ./src/doom --branching --matching "$(basename "$file")" &
	done
fi

