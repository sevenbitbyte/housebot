#!/bin/bash

PATH=$(dirname $(readlink -f $0))
PKGDIR=$(basename $PATH)

echo $PATH
echo $PKGDIR
