#!/bin/bash
set -e

if [ $# -ne 3 ]; then
	echo "usage: erofs_unpack <img_path> <mnt_path> <out_path>"
	exit 1
fi

img=$1
mnt=$2
out=$3
echo $mnt
echo $out

[ -d $mnt ] || rm -rf $mnt
[ -d $out ] || rm -rf $out
mkdir -p $mnt
mkdir -p $out

cur_dir=$(realpath $(dirname $0))
$cur_dir/erofsfuse $img $mnt
cp -ra $mnt/* $out/
fusermount -u $mnt
rm -rf $mnt

echo "erofs_unpack done"
