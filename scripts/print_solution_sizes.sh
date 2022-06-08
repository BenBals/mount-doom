#! /bin/bash

echo "execute me in your build folder"

echo ""
echo "file    solution_size"
for file in ../solutions/*; do
  solsize="$(wc -l <$file)"
  if (( $solsize > 0 )); then
    echo "$(basename $file) | $(wc -l <$file)"
  fi
done
