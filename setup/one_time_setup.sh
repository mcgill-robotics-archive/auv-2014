#!/bin/bash
############################
# .make.sh
# This script creates symlinks from the home directory to any desired dotfiles in ~/dotfiles
############################
source /opt/ros/hydro/setup.bash 
########## Variables

sudo cp -fv 48-RoboSub.rules /etc/udev/rules.d/48-RoboSub.rules
sudo cp -fv startup ~/startup
sudo cp -f startup.yml ~/.teamocil/startup.yml
sudo cp -f temperature.yml ~/.teamocil/temperature.yml

dir=$PWD   # dotfiles directory
olddir=~/dotfiles_old                 # old dotfiles backup directory
files="vimrc tmux.conf roboticrc"    # list of files/folders to symlink in homedir

##########

# create dotfiles_old in homedir
echo "Creating $olddir for backup of any existing dotfiles in ~"
mkdir -p $olddir
echo "...done"

# change to the dotfiles directory
echo "Changing to the $dir directory"
cd $dir
echo "...done"

# move any existing dotfiles in homedir to dotfiles_old directory, then create symlinks 
for file in $files; do
    echo "Moving any existing dotfiles from ~ to $olddir"
    mv ~/.$file ~/dotfiles_old/
    echo "Creating symlink to $file in home directory."
    ln -sv $dir/$file ~/.$file
done

cd $dir
cd ..

# Makes sure that the environment variables are in the ~./bashrc file.
if [[ `grep "export ROBOTIC_PATH" ~/.bashrc | wc -l` -eq 0 ]]; then
    echo 'ROBOTIC_PATH does not existe in bashrc, adding...'
    echo "export ROBOTIC_PATH=$PWD" >> ~/.bashrc
fi

if [[ `grep "source ~/.roboticrc" ~/.bashrc | wc -l` -eq 0 ]]; then
    echo 'roboticrc does not existe in bashrc, adding...'
    echo "source ~/.roboticrc" >> ~/.bashrc
fi 
