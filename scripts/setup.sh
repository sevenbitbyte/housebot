#!/bin/bash

PATH=$(dirname $(readlink -f $0))
PKGDIR=${PATH}
#echo ${PWD##*/}

echo $PATH
echo $PKGDIR
