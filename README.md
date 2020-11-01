# D* Lite

This is an implementation of the D* Lite algorithm, described [here](idm-lab.org/bib/abstracts/papers/aaai02b.pdf).
Currently only the version in Figure 3 is implemented, but the optimized version will be implemented shortly. Additionally, this implementation only searches for changed edge weights in successor nodes of the current "start" node.

The main function shows how to input custom testcases from a grid-based graph where adjacent cells are connected. It is represented by a 2D Vector of booleans, in which true means the cell can be entered and false means it cannot. The heuristic function for this graph is just the length of the shortest path between two nodes assuming the robot can enter any cell.
Here is a gif of the visualizer run on the sample test case. The green square is the robot, the red square is the goal, the black squares are obstacles the robot knows about, and the gray squares are obstacles the robot has not found yet:

![d-star-lite-vis](https://user-images.githubusercontent.com/40965890/97813114-b1708b80-1c4b-11eb-907b-e56a637bdcab.gif)

If you do not want to use the visualizer, there is another example shown in the main method. Note that you must define your own heuristic function.
The initial graph is shown here:
![initial](https://user-images.githubusercontent.com/40965890/97532869-40984d80-1985-11eb-9058-23707a3cff8b.png)
The "actual" graph is shown here:
![actual](https://user-images.githubusercontent.com/40965890/97533144-b2709700-1985-11eb-8f89-bc7d92f7a957.png)
Because the robot can only see changes in edge weights in successors of the current "start" node, the route the robot takes is re-planned twice: once when it reaches node #1 and once when it reaches node #3.

The test case is taken from [this video](https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2016/videos-for-advanced-lectures/advanced-lecture-1/).