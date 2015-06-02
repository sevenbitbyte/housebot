#!/bin/bash

PACKAGE=${PWD##*/}
STAGING_DIR=$(mktemp)
REMOTE_USER=vega
REMOTE_HOST=192.168.8.1
SSH_URL=$REMOTE_USER@$REMOTE_HOST

rm -rf $STAGING_DIR

git clone ./ $STAGING_DIR

tar -cf $STAGING_DIR.$PACKAGE.tar $STAGING_DIR

bzip2 -z $STAGING_DIR.$PACKAGE.tar

ssh $SSH_URL rm -rf /home/vega/Repos/$PACKAGE*
scp $STAGING_DIR.$PACKAGE.tar.bz2 $SSH_URL:/home/vega/Repos/$PACKAGE.tar.bz2
ssh $SSH_URL bzip2 -d /home/vega/Repos/$PACKAGE.tar.bz2
ssh $SSH_URL tar -xf /home/vega/Repos/$PACKAGE.tar
ssh $SSH_URL bash /home/vega/Repos/$PACKAGE/scripts/setup.sh
ssh $SSH_URL rm /home/vega/Repos/$PACKAGE.tar*
