# Author: Junhee Lee
# Date: 2024-09-05

# Install dependency
chmod 777 install_dependency.sh
./install_dependency.sh

# Be sure to use only bsw, lib, and rte that fit the repository.

# update bsw, lib and rte as SOTA
# git submodule update --remote --merge --init


# update lib's submodules ... version dependency exist
# cd src/lib 
# git submodule update --recursive --init 

chmod 777 install_*
./install_acados.sh
echo "-------------------------------- install acados done --------------------------------"

sleep 1s

./install_hpipm.sh
echo "-------------------------------- install hpipm done --------------------------------"
