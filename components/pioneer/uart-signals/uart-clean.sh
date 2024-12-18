#!/bin/bash

INPUT="$1"

if [ -z "$1" ]; then
  echo "No filename provided"
  exit 1
fi

grep -v 'X bits' "$INPUT" |sort -nk1 >$(basename "$INPUT")
