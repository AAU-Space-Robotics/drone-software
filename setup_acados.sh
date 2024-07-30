#!/bin/bash

# Change directory to acados directory
cd ~/drone-software/src/trajectory_gen/acados/

# Set environment variables
export acados_dir="$(pwd)"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$acados_dir/lib"
export ACADOS_SOURCE_DIR="$acados_dir"

# Change directory back to the original directory
cd ~/drone-software/
