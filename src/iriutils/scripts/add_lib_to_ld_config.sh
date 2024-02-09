#!/bin/bash

set -e

FILE="/etc/ld.so.conf.d/iri.conf"
LIB=
LOCAL="local/"

usage() {
  echo "Usage: $0 -l <library> [-p]"
  echo " -l  specify library name"
  echo " -p  specify if installed from package. Optional."
  exit 1
  }

while getopts ":hl:p" arg; do
  case $arg in
    l) LIB=$OPTARG;;
    p) LOCAL="";;
    h) usage;;
    *) usage;;
  esac
done

if [ -z "$LIB" ]
then
  echo "Error: no library specified"
  usage
fi

sudo touch $FILE
LINE="/usr/${LOCAL}lib/iri/${LIB}"
echo "Adding ${LINE} to ${FILE}..."
grep -qF -- "$LINE" "$FILE" || echo "$LINE" | sudo tee -a "$FILE" > /dev/null

sudo ldconfig

echo "Done."
