# Gazebo

## Information
- Version: gz only
- Programming: sdf, python

# Controllers in this folder  
There are three different folders at the moment: feagi_gazebo, gazebo_parser, and models (generic_controller, smart_car, taffy_bot).  

## feagi_gazebo  
Currently, there is nothing in it, but it uses student work from their capstone project, which is gazebo_parser. This is mainly used for the cloud to utilize and pass the SDF information.  

## gazebo_parser  
This design allows you to parse the SDF file into capabilities.json. The controller configurator is currently using this parser: https://brainsforrobots.com/configurator  

## generic_controller inside the model  
This is our best solution approach and the most dynamic design, where FEAGI can easily pick it up after the gazebo parser. It uses the `subprocess` library to communicate with FEAGI efficiently. It is designed to work with any SDF. This was created to work with students from the University of Pittsburgh. It is compatible with Mac, Windows (WSL), and Linux. This design is very accessible by anyone. 

# Set up instructions  
There are three different ways to install Gazebo. Fortunately, Gazebo supports all platforms. The Windows OS has a slightly unique setup for installing Gazebo since it uses WSL, which is essentially a Linux environment. So, technically, you are only installing Gazebo on Linux or Mac.

## Windows 11 set up
- wsl --install
- sudo apt-get update
- sudo apt-get install curl lsb-release gnupg
- sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
- echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
- sudo apt-get update
- sudo apt-get install gz-harmonic

## Mac (Apple Silicon)
- ulimit -n unlimited 
- /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)" 
- brew tap osrf/simulation
- brew install gz-ionic

## Linux (24.04)
- sudo apt-get update
- sudo apt-get install lsb-release gnupg
- sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
- echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
- sudo apt-get update
- sudo apt-get install gz-ionic

# Communication using protobuf
data coming from the gazebo and their topic has a documentation which is here:
https://gazebosim.org/api/msgs/11/install.html

https://gazebosim.org/docs/latest/getstarted/

https://gazebosim.org/api/transport/14/python.html

https://gazebosim.org/api/transport/14/installation.html

This allows you to obtain directly from the python, cpp, or protobuf. As right now, it's not an easy way to set up and build the gazebo without having a cmake to be built.


# How to test your new capabilities using smart car
1) cd to controllers/simulators/gazebo/models/smart_car
2) Run `gz sim -r free_world.sdf` (Mac has a different command, but you need to add '-r')
3) Open a new terminal and navigate to `controllers/simulators/gazebo/models/smart_car`
4) Paste your newly generated capabilities into capabilities.json
5) Run `python3 controller.py --ip FEAGI_IP_ADDRESS (NeuroRobotics Studio: Run `python3 controller.py --api_key "Paste_your_api_key_here")