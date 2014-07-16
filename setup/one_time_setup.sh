#!/bin/bash
############################
# .make.sh
# This script creates symlinks from the home directory to any desired dotfiles in ~/dotfiles
############################
source /opt/ros/hydro/setup.bash
########## Variables
echo "$(tput setaf 3)Copy udev.rules for serial devices$(tput sgr 0)"
sudo cp -fv 48-RoboSub.rules /etc/udev/rules.d/48-RoboSub.rules
echo "$(tput setaf 3)Copy Crontab startup script$(tput sgr 0)"
sudo cp -fv startup ~/startup
echo "$(tput setaf 3)Copy yml files for teamocil$(tput sgr 0)"
mkdir -p ~/.teamocil
cp -fv startup.yml ~/.teamocil/startup.yml
cp -fv controls.yml ~/.teamocil/controls.yml

dir=$PWD   # dotfiles directory
olddir=~/dotfiles_old                 # old dotfiles backup directory
files="vimrc tmux.conf roboticrc"    # list of files/folders to symlink in homedir

##########

# create dotfiles_old in homedir
mkdir -p $olddir

# change to the dotfiles directory
cd $dir

# move any existing dotfiles in homedir to dotfiles_old directory, then create symlinks
echo "$(tput setaf 3)Moving any existing dotfiles from ~ to $olddir and create symlinks $(tput sgr 0)"
for file in $files; do
    mv ~/.$file ~/dotfiles_old/
    ln -sv $dir/$file ~/.$file
done

cd $dir
cd ..

# Makes sure that the environment variables are in the ~./bashrc file.
if [[ `grep "export ROBOTIC_PATH" ~/.bashrc | wc -l` -eq 0 ]]; then
    echo "$(tput setaf 3)ROBOTIC_PATH does not exist in bashrc, adding... $(tput sgr 0)"
    echo "export ROBOTIC_PATH=$PWD" >> ~/.bashrc
fi

if [[ `grep "source ~/.roboticrc" ~/.bashrc | wc -l` -eq 0 ]]; then
    echo 'roboticrc does not exist in bashrc, adding...'
    echo "$(tput setaf 3)roboticrc is not sourced in bashrc, adding... $(tput sgr 0)"
    echo "source ~/.roboticrc" >> ~/.bashrc
fi
