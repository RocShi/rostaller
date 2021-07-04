#!/bin/bash
#
# @File          : run.sh
# @Version       : v0.1
# @Description   : This script is for installing ros-melodic automatically referring
#                  to http://wiki.ros.org/cn/melodic/Installation/Ubuntu. Please
#                  ensure you have configured the network as well as the proxy
#                  correctly before executing this script.
# @Author        : ShiPeng
# @Email         : RocShi@outlook.com
# @License       : MIT License
#
#    Copyright (c) 2021 ShiPeng
#
#    Permission is hereby granted, free of charge, to any person obtaining a copy
#    of this software and associated documentation files (the "Software"), to deal
#    in the Software without restriction, including without limitation the rights
#    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#    copies of the Software, and to permit persons to whom the Software is
#    furnished to do so, subject to the following conditions:
#
#    The above copyright notice and this permission notice shall be included in all
#    copies or substantial portions of the Software.
#
#    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#    SOFTWARE.
#

set -e

logfile=log.log
fifofile=run.fifo
rm -f "$fifofile"
mkfifo $fifofile
cat $fifofile | tee -a $logfile &
exec 3>&1
exec 4>&2
exec 1>$fifofile
exec 2>&1

echo
sleep 1

gbError="\033[1;31m[ERROR]\033[0m"
gbWarning="\033[1;33m[WARNING]\033[0m"
gbInfo="\033[1;32m[INFO]\033[0m"
gbGood="\033[1;32m[GOOD]\033[0m"

time=$(date "+%Y-%m-%d %H:%M:%S")
sudo echo -e "\n<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< $time >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"

# Step 1: Configure your Ubuntu repositories
# Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse."
# You can follow https://help.ubuntu.com/community/Repositories/Ubuntu for instructions
# on doing this. You should also select a more stable source url from your country at the
# same time, such as https://mirrors.tuna.tsinghua.edu.cn/ubuntu/
ConfRepo() {
    echo -e "$gbWarning . Please ensure you have configured ubuntu repositories and select a more stable source url from your country.\n"
}

# Step 2: set up sources.list
SetSrc() {
    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
    echo -e "$gbGood The sources.list has been set up. \n"
}

# Step 3: set up keys
SetKeys() {
    sudo apt install curl
    curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
    echo -e "$gbGood Keys have been set up. \n"
}

# Step 4: install ros
InstallRos() {
    sudo apt update
    sudo apt install ros-melodic-desktop-full
    echo -e "$gbGood The ros-melodic-desktop-full has been set up. \n"
}

# Step 5: set up environment
SetEnv() {
    echo "source /opt/ros/melodic/setup.bash" >>~/.bashrc
    source ~/.bashrc
    echo -e "$gbGood Environment has been set up. \n"
}

# Step 6: install dependencies for building packages
InstallDepend() {
    sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    echo -e "$gbGood Some dependencies for building packages have been installed. \n"
}

# Step 7: initialize rosdep
RosdepInit() {
    set +e
    sudo rosdep init

    if [ $? -eq 0 ]; then
        echo -e "$gbGood [rosdep init] was executed online successfully. \n"
    else
        echo -e "$gbError Could not execute [rosdep init] online, I will do this using rosdistro repository. \n"
        sudo rm -rf rosdistro
        echo -e "$gbInfo Trying to unzip local rosdistro repository ... \n"
        set -e
        tar -zxvf rosdistro.tar.gz 2>&1 >/dev/null
        cd rosdistro
        echo -e "$gbInfo Trying to update local rosdistro repository online ... \n"
        set +e
        git pull

        if [ $? -ne 0 ]; then
            current_commit=$(git rev-parse --short HEAD)
            echo -e "$gbWarning Could not update local rosdistro repository, commit on [$current_commit] will be used. \n"
        fi

        cd ..
        sudo mkdir -p /etc/ros/rosdep/sources.list.d
        sudo cp -f rosdistro/rosdep/sources.list.d/20-default.list /etc/ros/rosdep/sources.list.d/
        echo
        echo -e "$gbGood [rosdep init] was executed using rosdistro repository successfully. \n"
    fi
    set -e
}

# Step 8: update rosdep
RosdepUpdate() {
    set +e
    rosdep update

    if [ $? -eq 0 ]; then
        echo -e "$gbGood [rosdep update] was executed online successfully. \n"
    else
        set -e
        echo -e "$gbError Could not execute [rosdep update] online, I will do this using rosdistro repository. \n"

        url_original="https://raw.githubusercontent.com/ros/rosdistro/master"
        cd rosdistro
        url_local="file://$(pwd)"
        cd ..

        sudo cp -f /etc/ros/rosdep/sources.list.d/20-default.list /etc/ros/rosdep/sources.list.d/20-default.list.bkp
        sudo sed -i "s|${url_original}|${url_local}|g" /etc/ros/rosdep/sources.list.d/20-default.list

        sudo cp /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py.bkp
        sudo sed -i "s|${url_original}|${url_local}|g" /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py
        sudo cp /usr/lib/python2.7/dist-packages/rosdep2/rep3.py /usr/lib/python2.7/dist-packages/rosdep2/rep3.py.bkp
        sudo sed -i "s|${url_original}|${url_local}|g" /usr/lib/python2.7/dist-packages/rosdep2/rep3.py
        sudo cp /usr/lib/python2.7/dist-packages/rosdistro/__init__.py /usr/lib/python2.7/dist-packages/rosdistro/__init__.py.bkp
        sudo sed -i "s|${url_original}|${url_local}|g" /usr/lib/python2.7/dist-packages/rosdistro/__init__.py

        rosdep update

        sudo rm -rf rosdistro
        sudo mv /etc/ros/rosdep/sources.list.d/20-default.list.bkp /etc/ros/rosdep/sources.list.d/20-default.list
        sudo mv /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py.bkp /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py
        sudo mv /usr/lib/python2.7/dist-packages/rosdep2/rep3.py.bkp /usr/lib/python2.7/dist-packages/rosdep2/rep3.py
        sudo mv /usr/lib/python2.7/dist-packages/rosdistro/__init__.py.bkp /usr/lib/python2.7/dist-packages/rosdistro/__init__.py

        echo -e "$gbGood [rosdep update] was executed using rosdistro repository successfully. \n"
    fi
}

# Step 9: run demo
RunDemo() {
    echo -e "$gbGood Good job, bro., all tasks have been done! Let's enjoy ros now! \n"
    gnome-terminal --window -t roscore --execute roscore >/dev/null 2>&1 &&
        sleep 2 &&
        gnome-terminal --window -t turtlesim_node --execute rosrun turtlesim turtlesim_node >/dev/null 2>&1 &&
        gnome-terminal --window -t turtle_teleop_key --execute rosrun turtlesim turtle_teleop_key >/dev/null 2>&1 &&
        gnome-terminal --window -t rviz --execute rviz >/dev/null 2>&1 &&
        gnome-terminal --window -t rqt_graph --execute rqt_graph >/dev/null 2>&1
}

# main function
main() {
    ConfRepo
    SetSrc
    SetKeys
    InstallRos
    SetEnv
    InstallDepend
    RosdepInit
    RosdepUpdate
    RunDemo
}

main

printf "\015"
exec 1>&3
exec 2>&4
rm -f "$fifofile"
