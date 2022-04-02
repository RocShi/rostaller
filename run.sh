#!/bin/bash
#
# @File          : run.sh
# @Version       : v0.7
# @Description   : This script is for installing ROS 1 (indigo, kinetic,
#                  melodic and noetic) and ROS 2 galactic on corresponding
#                  ubuntu distributions automatically.
#                  Please ensure you have configured the network as well as the
#                  proxy correctly before executing this script.
# @Author        : ShiPeng
# @Email         : RocShi@outlook.com
# @License       : MIT License
#
#    Copyright (c) 2021-2022 ShiPeng
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

file_log=log
file_fifo=run.fifo
sudo rm -f "$file_fifo"
mkfifo $file_fifo
cat $file_fifo | tee -a $file_log &
exec 3>&1
exec 4>&2
exec 1>$file_fifo
exec 2>&1

echo
sleep 1

gbError="\033[1;31m[ERROR]\033[0m"
gbWarning="\033[1;33m[WARNING]\033[0m"
gbInfo="\033[1;32m[INFO]\033[0m"
gbGood="\033[1;32m[GOOD]\033[0m"

rosdistro="rosdistro-master-builtin"

ros_version=""

python_apt_version="python"
python_lib_version="python2.7"

current_time=$(date "+%Y-%m-%d %H:%M:%S")
sudo echo -e "\n <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< $current_time >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> \n"

# check if ros has been installed
CheckInstalledRos() {
    if [ -d "/opt/ros" ]; then
        while true; do
            installed_ros_list=$(echo $(ls /opt/ros) | sed 's/ /, /g' | sed 's/\(.*\), \(.*\)/\1 and \2/')
            echo -e "\n$gbWarning You have installed ros $installed_ros_list in your machine. Are you sure to continue? (yes/no) [no] \n"
            read input
            case $input in
            [yY][eE][sS] | [yY])
                return
                ;;
            [nN][oO] | [nN] | $null)
                echo -e "Bye! \n"
                exit 1
                ;;
            *)
                echo -e "\nInvalid input... \n"
                ;;
            esac
        done
    fi
}

# configure your Ubuntu repositories to allow "restricted," "universe," and
# "multiverse" by using "https://mirrors.tuna.tsinghua.edu.cn/ubuntu/" as the
# debian source
ChangeDebSrc() {
    echo
    echo -e "$gbInfo Your current system information is as follows:"
    cat /etc/os-release
    echo

    ubuntu_version=$(cat /etc/os-release | grep "VERSION_ID" | sed 's/\(VERSION_ID=\|"\|\.\)//g')

    case $ubuntu_version in
    1404)
        ros_version="indigo"
        ;;
    1604)
        ros_version="kinetic"
        ;;
    1804)
        ros_version="melodic"
        ;;
    2004)
        while true; do
            echo -e "\n$gbInfo Both ros 1 and ros 2 are supported by rostaller on ubuntu 20.04, which one do you want? (1/2) \n"
            read input
            case $input in
            1)
                ros_version="noetic"
                python_apt_version="python3"
                python_lib_version="python3"
                break
                ;;
            2)
                ros_version="galactic"
                break
                ;;
            *)
                echo -e "\nInvalid input... \n"
                ;;
            esac
        done
        ;;
    *)
        echo -e "$gbError Sorry, only ubuntu 14.04, 16.04, 18.04 and 20.04 are supported. \n"
        echo
        sudo rm -f "$file_fifo"
        exit 1
        ;;
    esac

    echo -e "\n$gbInfo The ros-$ros_version will be installed next. \n"

    if [ ! -f "/etc/apt/sources.list.bkp.rostaller" ]; then
        sudo cp /etc/apt/sources.list /etc/apt/sources.list.bkp.rostaller
    fi
    sudo cp SimpleSources/$ubuntu_version/sources.list /etc/apt/
    sudo apt-get update -y
    sudo apt-get upgrade -y
    echo -e "$gbGood The source has been updated and all softwares have been graded. \n"
}

# add ros source
AddRosSrc() {
    if [ $ros_version == "galactic" ]; then
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    else
        sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
    fi
    echo -e "$gbGood ROS source has been added. \n"
}

# set up keys
SetKeys() {
    sudo apt install curl gnupg lsb-release -y
    if [ $ros_version == "galactic" ]; then
        PrepareRosdistro
        sudo cp $rosdistro/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
    else
        curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
    fi
    echo -e "$gbGood Keys have been set up. \n"
}

# install ros
InstallRos() {
    sudo apt update -y
    if [ $ros_version == "galactic" ]; then
        sudo apt install ros-galactic-desktop -y
    else
        sudo apt install ros-$ros_version-desktop-full -y
    fi
    echo -e "$gbGood The ros-$ros_version-desktop-full has been installed. \n"
}

# install dependencies for building packages
InstallDepend() {
    sudo apt install $python_apt_version-rosdep $python_apt_version-rosinstall $python_apt_version-rosinstall-generator $python_apt_version-wstool build-essential -y
    echo -e "$gbGood Some dependencies for building packages have been installed. \n"
}

# initialize rosdep
RosdepInit() {
    set +e
    sudo rosdep init

    if [ $? -eq 0 ]; then
        echo -e "$gbGood [rosdep init] was executed online successfully. \n"
    else
        echo -e "$gbWarning Could not execute [rosdep init] online, I will do this using rosdistro repository. \n"
        PrepareRosdistro
        sudo mkdir -p /etc/ros/rosdep/sources.list.d
        cd $rosdistro
        sudo cp rosdep/sources.list.d/20-default.list /etc/ros/rosdep/sources.list.d/
        echo -e "$gbGood [rosdep init] was executed using rosdistro repository successfully. \n"
    fi
    set -e
}

# update rosdep
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

        sudo cp /usr/lib/$python_lib_version/dist-packages/rosdep2/gbpdistro_support.py /usr/lib/$python_lib_version/dist-packages/rosdep2/gbpdistro_support.py.bkp.rostaller
        sudo sed -i "s|${url_original}|${url_local}|g" /usr/lib/$python_lib_version/dist-packages/rosdep2/gbpdistro_support.py
        sudo cp /usr/lib/$python_lib_version/dist-packages/rosdep2/rep3.py /usr/lib/$python_lib_version/dist-packages/rosdep2/rep3.py.bkp.rostaller
        sudo sed -i "s|${url_original}|${url_local}|g" /usr/lib/$python_lib_version/dist-packages/rosdep2/rep3.py
        sudo cp /usr/lib/$python_lib_version/dist-packages/rosdistro/__init__.py /usr/lib/$python_lib_version/dist-packages/rosdistro/__init__.py.bkp.rostaller
        sudo sed -i "s|${url_original}|${url_local}|g" /usr/lib/$python_lib_version/dist-packages/rosdistro/__init__.py

        rosdep update

        sudo rm -rf master.zip
        sudo rm -rf rosdistro-master
        sudo mv /etc/ros/rosdep/sources.list.d/20-default.list.bkp.rostaller /etc/ros/rosdep/sources.list.d/20-default.list
        sudo mv /usr/lib/$python_lib_version/dist-packages/rosdep2/gbpdistro_support.py.bkp.rostaller /usr/lib/$python_lib_version/dist-packages/rosdep2/gbpdistro_support.py
        sudo mv /usr/lib/$python_lib_version/dist-packages/rosdep2/rep3.py.bkp.rostaller /usr/lib/$python_lib_version/dist-packages/rosdep2/rep3.py
        sudo mv /usr/lib/$python_lib_version/dist-packages/rosdistro/__init__.py.bkp.rostaller /usr/lib/$python_lib_version/dist-packages/rosdistro/__init__.py

        echo -e "$gbGood [rosdep update] was executed using rosdistro repository successfully. \n"
    fi
}

# set locale
SetLocale() {
    set +e
    locale

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale
    set -e
}

# prepare rosdistro: download from github or using local version
PrepareRosdistro() {
    set +e
    sudo rm -rf master.zip
    sudo rm -rf rosdistro-master
    sudo apt install wget unzip -y

    echo -e "\n$gbInfo Trying to download the latest rosdistro repository from github... \n"
    wget -T 10 https://github.com/ros/rosdistro/archive/refs/heads/master.zip

    if [ $? -eq 0 ]; then
        echo -e "$gbInfo Trying to unzip downloaded rosdistro repository... \n"
        unzip master.zip 2>&1 >/dev/null

        if [ $? -eq 0 ]; then
            echo -e "$gbInfo Latest rosdistro repository will be used. \n"
            rosdistro="rosdistro-master"
        else
            echo -e "$gbWarning Could not unzip downloaded rosdistro repository, so the local version will be used. \n"
            sudo rm -rf master.zip
            sudo rm -rf rosdistro-master
        fi
    else
        echo -e "$gbWarning Could not download the latest rosdistro repository, so the local version will be used. \n"
        sudo rm -rf master.zip
    fi
    set -e
}

# run demo
RunDemo() {
    echo -e "$gbGood Good job, bro., all tasks have been done! Run demo now? (yes/no) [yes] \n"

    while true; do
        read input

        case $input in

        [yY][eE][sS] | [yY] | $null)
            echo -e "\nLet's enjoy ros! \n"

            if [ $ros_version == "galactic" ]; then
                gnome-terminal -- bash -c "source /opt/ros/galactic/setup.bash; ros2 run demo_nodes_cpp talker" >/dev/null 2>&1 &&
                    sleep 2 &&
                    gnome-terminal -- bash -c "source /opt/ros/galactic/setup.bash; ros2 run demo_nodes_py listener" >/dev/null 2>&1
            else
                gnome-terminal -- bash -c "source /opt/ros/$ros_version/setup.bash; roscore" >/dev/null 2>&1 &&
                    sleep 5 &&
                    gnome-terminal -- bash -c "source /opt/ros/$ros_version/setup.bash; rviz" >/dev/null 2>&1 &&
                    sleep 2 &&
                    gnome-terminal -- bash -c "source /opt/ros/$ros_version/setup.bash; rosrun turtlesim turtlesim_node" >/dev/null 2>&1 &&
                    sleep 2 &&
                    gnome-terminal -- bash -c "source /opt/ros/$ros_version/setup.bash; rosrun turtlesim turtle_teleop_key" >/dev/null 2>&1 &&
                    sleep 2 &&
                    gnome-terminal -- bash -c "source /opt/ros/$ros_version/setup.bash; rqt_graph" >/dev/null 2>&1
            fi

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

# install ros 1
InstallRos1() {
    AddRosSrc
    SetKeys
    InstallRos
    InstallDepend
    RosdepInit
    RosdepUpdate
}

# install ros 2
InstallRos2() {
    SetLocale
    SetKeys
    AddRosSrc
    InstallRos
}

# main function
main() {
    CheckInstalledRos
    ChangeDebSrc

    if [ $ros_version == "galactic" ]; then
        InstallRos2
    else
        InstallRos1
    fi

    RunDemo
}

main

printf "\015"
exec 1>&3
exec 2>&4
sudo rm -f "$file_fifo"
