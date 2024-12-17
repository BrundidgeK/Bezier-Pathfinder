# Bezier-Pathfinder
Cubic Beizer Curve Pathfinding Algorithm

⭐ Star the Project — it motivates us to move forward!

Bezier-Pathfinder is a Unity project built in C#, serving to simulate and prototype how Navigation Bezier (NaviBezi) operates in given certain environmental conditions and limitations. It allows users to interact with NaviBezi by changing its parameters and the environment while Unity visualizes the results. NaviBezi is a pathfinding algorithm designed to provide smooth and optimal paths using cubic bezier curves. It takes the start and end points (the curve's main points) and the environmental obstacles to calculate the best positions for the control points. The algorithm utilizes well-known functions from A*, Dijkstra's Algorithm, and other pathfinding algorithms to find the most efficient path while avoiding any obstacles. It has applications in AI, robotics, and game development. 

![Alt text](https://github.com/BrundidgeK/GIF/blob/main/NaviBezi%20Demo.gif)

# Features
- Beizer Curve Path Planning: Generates paths using cubic bezier curves to avoid abrupt turns and corners
- NaviBezi Interaction: Modify the environment and fine-tune the curve parameters in real time to observe dynamic results.
- Visualization: Visualize the generated paths in 2D

# Requirements
- Unity 2022.3.50f1 or later
- Windows or macOS

# Installation
1. Clone the repository:
``` Bash
git clone https://github.com/BrundidgeK/Bezier-Pathfinder.git  
```
2. Open the project in the Unity Editor (Recommended Version: 2022.3.50f1 or later)
3. Play the scene and start exploring NaviBezi's pathfinding capabilities!

# Usage
- Environment Interaction: Modify obstacles by adding vertices and changing shapes or add new obstacles with the Obstacle Prefab
- Use the control panel to tweak the behavior of the algorithm:

Parameter | Description
--- | --- 
T Resolution | Controls curve smoothness and collision-check intervals  (1/t resolution = number of points on curve)
Max Iterations | Maximum number of loops for the algorithm to optimize the control points
Radius | The curve thickness; defines the minimum safe distance from obstacles
Initial Distance | The starting search distance around control points for optimal placement
Decay Rate | Linearly decreases the search distance (as a percentage) during optimization
Circle Segments | Number of points on the search circle evaluated during control point optimization

# Performance Disclaimer
Be wary that certain parameter values can decrease performance because they increase the path's complexity and the computational power required to find a path. 
- High values for Iterations, Circle Segments, and Radius may significantly impact performance.
- Low values for t Resolution result in smoother paths but require more computational checks.
Optimize parameters based on your system's performance and environment complexity.

# Future Work
- Depending on certain factors of the environment, NaviBezi requires high computational power for its calculations. To mitigate the long runtime, NaviBezi will be written in C/C++
- NaviBezi will be expanded to chain multiple curves together, instead of only using one for the whole path
- Potential for implementation in FIRST Tech Challenge's Autonomous Period

# Contributing 
Contributions are welcome! If you'd like to improve the algorithm, fix bugs, or expand the project:
1. Fork the repository
2. Create a new branch:
   ```Bash
   git checkout -b feature-name  
4. Commit your changes and push to your fork
5. Submit a detailed Pull Request, stating what was changed/added and why

Highly suggest you look into the [document](https://docs.google.com/document/d/1cNlrEZWTvh921VXQtFRHHvun9Ap2JBVG0WgolvAt2Vo/edit?usp=sharing) that explains how the NaviBezi algorithm works.

# Links/Resources
-  [Breakdown of NaviBezi's Algorithm](https://docs.google.com/document/d/1cNlrEZWTvh921VXQtFRHHvun9Ap2JBVG0WgolvAt2Vo/edit?usp=sharing)
-  [Bezier-Pathfiner on Itch.io](https://wishkish.itch.io/navibezi)
