#!/bin/bash
#
# @File          : run.sh
# @Version       : v0.3
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
sudo rm -f "$fifofile"
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

rosdistro="rosdistro-master-9cc7b9d"

time=$(date "+%Y-%m-%d %H:%M:%S")
sudo echo -e "\n<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< $time >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n"

# Step 1: configure your Ubuntu repositories to allow "restricted," "universe,"
# and "multiverse" by using "https://mirrors.tuna.tsinghua.edu.cn/ubuntu/" as the
# debian source
ChangeDebSrc() {
    SystemVersion=$(cat /etc/os-release | grep "VERSION_ID" | sed 's/VERSION_ID=//')
    if [ "$SystemVersion" != \""18.04"\" ]; then
        echo -e "$gbError Sorry, only Ubuntu 18.04 was supported. The current system information is as follows: \n"
        cat /etc/os-release
        echo
        sudo rm -f "$fifofile"
        exit 1
    fi
    if [ ! -f "/etc/apt/sources.list.bkp.rostaller" ]; then
        sudo cp /etc/apt/sources.list /etc/apt/sources.list.bkp.rostaller
    fi
    sudo cp SimpleSources/sources.list /etc/apt/
    sudo apt-get update -y
    sudo apt-get upgrade -y
    echo -e "$gbGood The source has been updated and all softwares have been graded.\n"
}

# Step 2: add ros source
AddRosSrc() {
    sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
    echo -e "$gbGood ROS source has been added. \n"
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
    echo -e "$gbGood The ros-melodic-desktop-full has been installed. \n"
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
        echo -e "$gbWarning Could not execute [rosdep init] online, I will do this using rosdistro repository. \n"

        sudo rm -rf master.zip
        sudo rm -rf rosdistro-master
        sudo apt install wget unzip -y

        echo -e "\n$gbInfo Trying to download the latest rosdistro repository from github... \n"
        wget -T 10 https://github.com/ros/rosdistro/archive/refs/heads/master.zip

        if [ $? -eq 0 ]; then
            echo -e "$gbInfo Trying to unzip downloaded rosdistro repository... \n"
            unzip master.zip 2>&1 >/dev/null

            if [ $? -eq 0 ]; then
                echo -e "$gbInfo Yaml files from the latest rosdistro repository will be used. \n"
                rosdistro="rosdistro-master"
            else
                echo -e "$gbWarning Could not unzip downloaded rosdistro repository, so the local yaml files will be used. \n"
                sudo rm -rf master.zip
                sudo rm -rf rosdistro-master
            fi
        else
            echo -e "$gbWarning Could not download the latest rosdistro repository, so the local yaml files will be used. \n"
            sudo rm -rf master.zip
        fi

        sudo mkdir -p /etc/ros/rosdep/sources.list.d
        cd $rosdistro
        sudo cp rosdep/sources.list.d/20-default.list /etc/ros/rosdep/sources.list.d/
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
        echo -e "$gbWarning Could not execute [rosdep update] online, I will do this using rosdistro repository. \n"

        url_original="https://raw.githubusercontent.com/ros/rosdistro/master"
        url_local="file://$(pwd)"
        cd ..

        sudo cp -f /etc/ros/rosdep/sources.list.d/20-default.list /etc/ros/rosdep/sources.list.d/20-default.list.bkp.rostaller
        sudo sed -i "s|${url_original}|${url_local}|g" /etc/ros/rosdep/sources.list.d/20-default.list

        sudo cp /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py.bkp.rostaller
        sudo sed -i "s|${url_original}|${url_local}|g" /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py
        sudo cp /usr/lib/python2.7/dist-packages/rosdep2/rep3.py /usr/lib/python2.7/dist-packages/rosdep2/rep3.py.bkp.rostaller
        sudo sed -i "s|${url_original}|${url_local}|g" /usr/lib/python2.7/dist-packages/rosdep2/rep3.py
        sudo cp /usr/lib/python2.7/dist-packages/rosdistro/__init__.py /usr/lib/python2.7/dist-packages/rosdistro/__init__.py.bkp.rostaller
        sudo sed -i "s|${url_original}|${url_local}|g" /usr/lib/python2.7/dist-packages/rosdistro/__init__.py

        rosdep update

        sudo rm -rf master.zip
        sudo rm -rf rosdistro-master
        sudo mv /etc/ros/rosdep/sources.list.d/20-default.list.bkp.rostaller /etc/ros/rosdep/sources.list.d/20-default.list
        sudo mv /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py.bkp.rostaller /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py
        sudo mv /usr/lib/python2.7/dist-packages/rosdep2/rep3.py.bkp.rostaller /usr/lib/python2.7/dist-packages/rosdep2/rep3.py
        sudo mv /usr/lib/python2.7/dist-packages/rosdistro/__init__.py.bkp.rostaller /usr/lib/python2.7/dist-packages/rosdistro/__init__.py

        echo -e "$gbGood [rosdep update] was executed using rosdistro repository successfully. \n"
    fi
}

# Step 9: run demo
RunDemo() {
    sudo apt install xdotool -y

    echo -e "$gbGood Good job, bro., all tasks have been done! Run demo now? [yes/no] \n"

    while true; do
        read input

        case $input in

        [yY][eE][sS] | [yY] | $null)
            echo -e "\nLet's enjoy ros! \n"

            gnome-terminal --window -t roscore >/dev/null 2>&1
            WID=$(xdotool search --name "roscore" | head -1)
            xdotool windowfocus $WID
            xdotool type --clearmodifiers 'roscore'
            xdotool key Return
            sleep 5

            gnome-terminal --window -t rviz >/dev/null 2>&1
            WID=$(xdotool search --name "rviz" | head -1)
            xdotool windowfocus $WID
            xdotool type --clearmodifiers 'rviz'
            xdotool key Return
            sleep 5

            gnome-terminal --window -t turtlesim_node >/dev/null 2>&1
            WID=$(xdotool search --name "turtlesim_node" | head -1)
            xdotool windowfocus $WID
            xdotool type --clearmodifiers 'rosrun turtlesim turtlesim_node'
            xdotool key Return
            sleep 5

            gnome-terminal --window -t turtle_teleop_key >/dev/null 2>&1
            WID=$(xdotool search --name "turtle_teleop_key" | head -1)
            xdotool windowfocus $WID
            xdotool type --clearmodifiers 'rosrun turtlesim turtle_teleop_key'
            xdotool key Return
            sleep 5

            gnome-terminal --window -t rqt_graph >/dev/null 2>&1
            WID=$(xdotool search --name "rqt_graph" | head -1)
            xdotool windowfocus $WID
            xdotool type --clearmodifiers 'rqt_graph'
            xdotool key Return
            sleep 5

            break
            ;;

        [nN][oO] | [nN])
            echo -e "\nBye! \n"
            break
            ;;

        *)
            echo -e "\nInvalid input... \n"
            ;;
        esac
    done
}

# main function
main() {
    ChangeDebSrc
    AddRosSrc
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
sudo rm -f "$fifofile"
