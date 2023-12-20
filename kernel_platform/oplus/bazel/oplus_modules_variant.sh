#!/bin/sh
set -x
targets=$1
variant=$2
script_dir=$(dirname $(readlink -f "$0"))
file=$script_dir/"oplus_modules_variant.bzl"
echo "targets  $targets variant $variant file $file"
echo "bazel_support_target = [\"$targets\"]" > $file
echo "bazel_support_variant = [\"$variant\"]" >> $file