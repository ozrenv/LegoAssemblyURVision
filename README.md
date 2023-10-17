# Repository Description

[![Click to watch the video](https://github.com/ozrenv/LegoAssemblyURVision/blob/master/youtube.png)](https://www.youtube.com/watch?v=t85DS-9UNHM)
Click the image to watch the video.

This repository contains a simulation with two integral parts: image processing and robot programming using RoboDK. 
The image processing program, `main.py`, takes an input image taken with a camera of bricks on the workspace and generates five CSV files with brick information (x, y coordinates, and orientation).
The CSV files are inputed into the RoboDK program, which uses the brick information to instruct the building of 4 Simpson figures on the workspace.

### Dependencies
To run the simulation and the Python program, ensure the following is installed:

- [RoboDK](https://robodk.com/)
- [PyCharm](https://www.jetbrains.com/pycharm/) (or a similar IDE)
- Python packages: OpenCV, Numpy, Pandas, Csv


