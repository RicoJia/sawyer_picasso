# Sawyer the Picasso

## Project Overview
A robot can draw like an artist! In this project, you are able to draw a custom picture on a piece of letter size paper using a Rethink Sawyer Robot. 
The package implements the drawing process on a pixel by pixel basis, using [Depth-First-Search algorithm](https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/).  
The drawing quality is further enhanced by a force control feature, so don't worry about bumpy or slanted surfaces!    

##### Authors
Rico Jia - ruotongjia2020@u.northwestern.edu

Emek Baris Kucuktabak

##### System Requirement
Ros Melodic

##### Video Demonstration
[![Screenshot from 2020-01-01 17-35-32](https://user-images.githubusercontent.com/39393023/71647478-2934f900-2cbd-11ea-9d6a-d6016e724da1.png)](https://youtu.be/oZADOFXaMHI)

## Usage

* Prepare a drawing area with:
    - A stable table with a height around 0.75m. 
    - A piece of letter size paper or a dry-erase board
    - A marker with tip size 1.25"x5.13" 
    - Two clamp-ons/Scotch tape for stablizing the dry-erase board/paper
 
* Start the Sawyer arm and its connection to the control computer
 
* Find the center of the workspace by running:
```
rosrun intera_examples go_to_joint_angles.py -q -0.715609375 -0.397354492188 -2.61908300781 -1.89218652344 -1.25625976563 -0.537494140625 -2.27596777344                      
```

* Set up the drawing area 2.0cm below the end gripper position. Don't worry about getting the exact vertical position, the force control feature
will compensate for the inaccuracy in the vertical direction

* Restart Sawyer and rebuild connection between the control computer and Sawyer robot

* Download this package to a local computer and build
``` 
$ mkdir -p ~/sawyer_picasso/src
$ cd ~/sawyer_picasso/src
$ wstool init
$ wstool set sawyer_picasso --git https://github.com/RicoJia/sawyer_picasso.git
$ wstool update
$ cd ..
$ catkin_make 
```
* Replace the image in ```image``` with a custom image, then rename your image file to _sawyer_image.format_. 
Supported file formats are: .JPEG2000, .JPEG, .png, .tiff, .bmp 

* Run the project on terminal:
``` 
$ cd ~/sawyer_picasso
$ source devel/setup.bash
$ rosrun sawyer_picasso Sawyer_Picasso
```

## Process Structure

There are 4 stages in the entire process
* Image Processing
* Path Planning using Depth-First-Search Algorithm
* Robot arm trajectory planning
* Trajectory execution with force control 

![Screenshot from 2020-01-02 13-11-46](https://user-images.githubusercontent.com/39393023/71686703-88e4e000-2d61-11ea-9831-f9a738a3e8dc.png)

For more details of each stage, please check out the doc strings of
 ```pic_2_array.py```, ```path_planner.py```, ```trajectory_generation.py```, 
 ```Sawyer_Picasso```
 
 
