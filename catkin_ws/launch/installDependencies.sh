#!/bin/bash

echo "About to install tmux which will allow you to have multiple pannels in a terminal.";
sudo add-apt-repository ppa:pi-rho/dev
sudo apt-get update
sudo apt-get install tmux
echo "About to install the utility software used to automatically SSH from script.";
sudo apt-get install sshpass
echo "About to install the utility software used to automatically launch tmux terminals.";
gem install teamocil
