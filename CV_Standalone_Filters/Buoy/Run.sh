#!/bin/bash

# Get shell file directory (project directory)
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Make the sources if missing
if [ ! -f $DIR/bin/CV-Buoy ]; then
	(cd $DIR && make)
fi;

# Run the project
(cd $DIR && ./bin/CV-Buoy)
