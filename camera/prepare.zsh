#!/usr/bin/env zsh

#usage: ./prepare '<commit message>'
# DO NOT FORGET THE '' QUOTES

# This checks that you have one command line argument
if [ $# -ne 1 ]
then
	echo 'Did you read the file before you ran it? -_-'
	exit 0
fi

git add depth_images/*
git add point_clouds/*
git add images/*

git commit -m "$1"

echo 'Done. All have been committed. Please push'
