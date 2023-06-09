# ENPM 661

## A* ALGORITHM

### Student Name
- Shantanu Parab
- Vineet Singh

### Directory ID & UID
- sparab - 119347539
- vsingh03 - 119123614

## Libraries used are: 
>numpy, argparse, timeit, queue, OpenCV, math 

## Source Code Files
- [`a_star_shantanu_vineet.py`](a_star_shantanu_vineet.py) - Contains the implementation of Dijkstra algorithm to solve the path planning problem.
  + The program will run a dijkstra algorithm for path planning and create a video. This video will be saved after execution.
  + Once the program runs  completetly it will show  a image output of optimum path.
  + Press enter once the image is shown this will complete the video recording
  + The program will ask the user to give the inputs for start and goal and check wheter it is valid

Note: The output video is created as Astar01.avi which is converted to mp4 using another software.

  To test any other initial state and goal state, provide the states to --InitState and --GoalState parameter while running the code. Same as in step 3 i.e. (x y theta)

  The canvas size is 600*250 in Cartesian. 
  The inputs of initial and goal nodes should be given in Cartetsian Coordinates, and Step size between 1 to 10. 


## How to Run the Program:
To run the program, open the command prompt/terminal and navigate to the directory where the source code files are located. Then, type the following command: 

    python3 a_star_shantanu_vineet.py --InitState 50 125 0 --GoalState 430 230 0 --StepSize 10 --RobotClearance 5 --ObjectClearance 5
    python3 a_star_shantanu_vineet.py --InitState 30 30 0 --GoalState 400 40 0 --StepSize 10 --RobotClearance 5 --ObjectClearance 5


## Github Link
[Repository](https://github.com/shantanuparabumd/ASTAR.git)

## Video Output
[Output Video](Astar01.mp4)




