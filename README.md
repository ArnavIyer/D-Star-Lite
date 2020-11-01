# D* Lite

NOTE: Currently using the visualizer may not work, a bug has been found.

This is an implementation of the D* Lite algorithm, described [here](idm-lab.org/bib/abstracts/papers/aaai02b.pdf).
Currently only the version in Figure 3 is implemented, but the optimized version will be implemented shortly. Additionally, this implementation only searches for changed edge weights in successor nodes of the current "start" node.

In the main function, there is a sample test case of a simple graph. The heuristic function for two nodes in this graph is the amount of nodes between them inclusive of the two nodes.
The initial graph is shown here:
![initial](https://user-images.githubusercontent.com/40965890/97532869-40984d80-1985-11eb-9058-23707a3cff8b.png)
The "actual" graph is shown here:
![actual](https://user-images.githubusercontent.com/40965890/97533144-b2709700-1985-11eb-8f89-bc7d92f7a957.png)
Because the robot can only see changes in edge weights in successors of the current "start" node, the route the robot takes is re-planned twice: once when it reaches node #1 and once when it reaches node #3.

After the above test, the main function also shows how to input custom testcases from a grid-based graph where adjacent cells are connected. It is represented by a 2D Vector of booleans, in which true means the cell can be entered and 0 means it cannot.

The test case is taken from [this video](https://ocw.mit.edu/courses/aeronautics-and-astronautics/16-412j-cognitive-robotics-spring-2016/videos-for-advanced-lectures/advanced-lecture-1/).