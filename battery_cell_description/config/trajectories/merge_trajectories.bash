#!/bin/bash

# Define the output file
path=""
if [ ! -z $1 ]
then
      path=$1
else
      path="."
fi

output_file="$path/trajectories.yaml"

# Empty the output file
> "$output_file"
echo $path/*.yaml
# Loop through all .yaml files in the current directory
for file in $path/*.yaml; do
    # Check if the file is not the output file
    if [ "$file" != "$output_file" ]; then
        # Append the content of the YAML file to the output file
        cat "$file" >> "$output_file"
        # Optionally add a newline between files if needed
        echo "" >> "$output_file"
    fi
done

echo "All YAML files have been merged into $output_file."
