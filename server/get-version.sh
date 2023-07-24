#!/bin/bash
packedge_file_path="./package.json"
version_file_path="./version.json"

version=$(grep -o '"version": *"[^"]*"' $packedge_file_path | cut -d '"' -f 4)

sed -i "s/<version>.*<\/version>/<version>${version}<\/version>/" ../package.xml

# for ci pipeline
cp ../package.xml ../version.xml

echo "Read package.json at $packedge_file_path"

line_num=1
while read line; do
if [[ "$line" == *"version"* ]];
then
  test=`echo "$line" | tr ',' ' '`
  echo "Find version: {$test}"
  echo "{$test}" >| $version_file_path
fi
line_num=$((line_num+1))
done < $packedge_file_path