#!/bin/bash

PATH=$(dirname $(readlink -f $0))
PKGDIR=$(dirname "$PATH")

echo $PKGDIR