#!/bin/sh
## @junhee.lee - 2024

echo "Installation of acados..."

set -e
cd acados

python3 -m pip install --upgrade pip

# Check the state of the build directory
if [ -d "build" ]; then
    if [ -f "build/Makefile" ]; then
        echo "Makefile exists in build directory. Skipping rebuild."
        exit 0  # Exit the script
    else
        if [ -z "$(ls -A build)" ];then
            echo "Build directory exists but is empty. Proceeding to build."
        else
            echo "Build directory exists and is not empty, but Makefile is missing. Proceeding to build."
        fi
    fi
else
    echo "Build directory does not exist. Creating build directory."
    mkdir -p build
fi

# build
cd build
echo "Build acados..."

cmake -DACADOS_WITH_QPOASES=ON ..
# -DACADOS_WITH_QPOASES=ON ..
# -DACADOS_WITH_QORE=ON
# -DACADOS_WITH_OSQP=ON
# -DACADOS_WITH_QPDUNES=ON
# -DACADOS_WITH_HPMPC=ON
make install -j6

echo "Build acados done!!"
cd ..


echo "Install python interface..."

# python interface installation
pip install --use-pep517 -e ./interfaces/acados_template
export ACADOS_SOURCE_DIR="$(pwd)"  

pip install casadi==3.6.5 numpy==1.24.4 scipy==1.10.1 pandas==2.0.3

# Register environment variables in .bashrc
BASHRC_FILE="$HOME/.bashrc"
LD_LIBRARY_PATH_LINE="export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\"$(pwd)/lib\""
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/lib
ACADOS_SOURCE_DIR_LINE="export ACADOS_SOURCE_DIR=\"$(pwd)\""
export ACADOS_SOURCE_DIR=$(pwd)

# source .bashrc
source $BASHRC_FILE

# Remove any existing commented or uncommented LD_LIBRARY_PATH or ACADOS_SOURCE_DIR lines that contain 'acados'
sed -i '/^#.*LD_LIBRARY_PATH=.*acados.*/d' $BASHRC_FILE
sed -i '/^export LD_LIBRARY_PATH=.*acados.*/d' $BASHRC_FILE
sed -i '/^#.*ACADOS_SOURCE_DIR=.*acados.*/d' $BASHRC_FILE
sed -i '/^export ACADOS_SOURCE_DIR=.*acados.*/d' $BASHRC_FILE

# Add new lines if they don't already exist
grep -qxF "$LD_LIBRARY_PATH_LINE" $BASHRC_FILE || echo "$LD_LIBRARY_PATH_LINE" >> $BASHRC_FILE
grep -qxF "$ACADOS_SOURCE_DIR_LINE" $BASHRC_FILE || echo "$ACADOS_SOURCE_DIR_LINE" >> $BASHRC_FILE

# Running Python example
# Echo for automatically install terralenderer, Timeout for automatically terminate the process
echo "y" | timeout 20s python3 ./examples/acados_python/getting_started/minimal_example_ocp.py 
rm acados_ocp.json

echo "Installation complete if the example runs fine!"
