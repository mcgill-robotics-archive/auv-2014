#!/bin/bash
echo start copying...
for file in *
do
	ln -s $file ~/
done
echo done.
