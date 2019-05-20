#!/usr/bin/env bash
n=0
for file in *.jpg ; do mv  "${file}" $1"${n}".jpg; n=$((n+1));  done
n=0
for file in *.txt ; do mv  "${file}" $1"${n}".txt; n=$((n+1));  done
