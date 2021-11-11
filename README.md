
# Plannie

Plannie is a framework to path planning in Python, simulated environments, and flights in real environments.


## Installation

The project can be installed in any directory on the computer with the following commands: 

```bash
  git clone https://github.com/lidiaxp/plannie.git
  pip install -r requirements.txt
```
    
## Features

- Classical, meta heuristic, and machine learning path planning techniques in 2D and 3D;
- Integration with Python, Gazebo ([MRS Simulator](https://ctu-mrs.github.io/) and [Sphinx](https://developer.parrot.com/docs/sphinx/whatissphinx.html)), and real environmts
- Avoid dynamic obstacles;
- Deep benchmark with path length, time, memory, CPU, battery, fligth time, among others;
- Supports several external maps, as [House Expo](https://github.com/TeaganLi/HouseExpo) and [Video Game maps](https://www.movingai.com/benchmarks/voxels.html);
- Supports Hokuyo, RpLidar, Velodyne, GPS, GPS RTK, and several cameras;
- Travelling salesman problem and coverage path planning algorithms;
- Decision maker supports modular localization, mapping, controller, and vision scripts;
- Decision maker suport multi-UAVs.

The path planning algorithms avaiable are: 

- **Exact classical**: A*, Artificial Potential Field
- **Aproximate classical**: Probabilistic Road Map, Rapid-Exploring Random Tree, Rapid-Exploring Random Tree - Connect
- **Meta heuristic**: Particle Swarm Optimization, Grey Wolf Optimization, Glowworm Swarm Optimization
- **Machine Learning**: Q-Learning

## Running Tests

```diff
- Please, read Mode of Use Section before running tests
```

Note: This planner is based in MRS, to change is just need switch controller (explained below) and change the subscribers topics.

The environment is defined in helper/ambiente.py to 2D environments and in 3D/helper/ambiente.py to 3D environmets.

#### To run 2D tests in Python:

```bash
  python3 unknownEnvironment.py
```

#### To run 2D tests in simulator and real environments:

```bash
  python3 movimentoROS.py
```

If use unknown environments (the default uses rplidar) run this code before, in other terminal:

```bash
  python3 helper/rplidar.py
```

Else, is needed change the variable ```self.knownEnvironment``` from 0 to 1, and follow the instructions in Mode of Use to configure the environment.

#### To run 3D tests in Python:

```bash
  cd 3D
  python3 unknownEnvironment3D.py
```

#### To run 3D tests in simulator and real environments:

```bash
  cd 3D
  python3 movimentoROS3D.py
```

If you will use known environments, is needed change the variable ```self.knownEnvironment``` from 0 to 1, and follow the instructions in Mode of Use to configure the environment.

## Mode of Use

In the movimentoROS* codes: 

- Any environment can be used to run in Gazebo, but is needed that your environement is opened before run Plannie. In Plannie, the control is defined in ```andarGlobal``` function at ```utilsUav``` file. By default, the controller is MPC used by MRS, to work with ```/cmd_vel``` change the variable ```self.controller``` to 0 (1: MPC in MRS | 0: cmd_vel);
- To change between known e unknown environment change the variable ```self.knownEnvironment```. *Note*: If use known environment is needed configure the environment, as shown in Section ```Change Environment``` below. If use unknown environment is import define start and goal node, as described in the Section below. *Note2*: By default the mapping is made with velodyne in 3D environment and rplidar in 2D environment;
- To use rplidar is needed download [this package](https://github.com/tysik/obstacle_detector) on your ROS workspace and compile the workspace;
- It is possible choose the path planning technique in the imports, the strucutre is similar to these model and can be change to any path planning algorithm present in the plannie;
- You can add other variables similar to this to use different algorithms to initial planning and when discover new obstacles. The technique used in this cases are defined in callbackStatic (when discover new environments) and callbackBuildMap (initial planning);
- The dynamic path planning are disabled to simulator, but if you need use just modify the callbackDynamic.

In the unknownEnvironment* codes:

- There are a list of algorithms avaiable at the start (next to line 25), just turn the variable true to use the technique;
- In this case, by default, is not possible use different algorithms to initial planning and when discover new obstacles. However, it is possible import the technique in helper/unknown.py and define the best technique to each planning. The initial planning is defined in alg.run next to line 96, and the planning when discover new obstacles is defined in alg.run nexto to line 326;
- Define dynamic obstacles in helper/unknown.py in the variables obs* (next to line 22);
- The algorithm to be used in dynamic path planning is defined with the function newSmooth nexto to line 276 and 279. This function uses Pedestrian Avoidance Method, if you need use Riemannian Motion Policies, import it at start and switch this function.

If you will carry out flights in real environment, have several scripts to support use the sensors in the folder _helper_.

## Change Environment

To change 2D environment use the file ```helper/ambiente.py``` and modify these variables:

- **tamCapa**: define the size of risk zone
- **self.limiar**: define the environment size
- **self.xs**: define the start node in X
- **self.ys**: define the start node in Y
- **self.xt**: define the goal node in X
- **self.yt**: define the goal node in Y
- **obsVx**: define x-axis of vertical obstacles
- **obsVy**: define y-axis of horizontal obstacles
- **obsHx**: define x-axis of vertical obstacles
- **obsHy**: define y-axis of horizontal obstacles

The difference between vertical and horizonal obstacles just matter if use risk zone.

To change 3D environment use the file ```3D/helper/ambiente.py``` and modify these variables:

- **tamCapa**: define the size of risk zone
- **self.limiar**: define the environment size
- **self.xs**: define the start node in X
- **self.ys**: define the start node in Y
- **self.ys**: define the start node in Z
- **self.zt**: define the goal node in X
- **self.yt**: define the goal node in Y
- **self.zt**: define the goal node in Z
- **vermelhoX**: define x-axis of walls that the vehicle can not pass 
- **vermelhoY**: define y-axis of walls that the vehicle can not pass 
- **rosaX**: define x-axis of walls that the vehicle just go over 
- **rosaY**: define y-axis of walls that the vehicle just go over 
- **amareloX**: define y-axis of walls that the vehicle just go under 
- **amareloY**: define y-axis of walls that the vehicle just go under

To modify the height of these walls change zobs* variables. If you want define the obstacles nodes separately, ignore the variables [vermelhoX, vermelhoY, rosaX, rosaY, amareloX, amareloY] and change:

- **self.xobs**: define all X nodes
- **self.yobs**: define all Y nodes
- **self.zobs**: define all Z nodes

To use external maps:
- **houseExop maps**: run readFromHouseExpo function in ```helper/ocupancyGridMap.py``` (defining the json file form houseExpo) and import the return in the ambiente.py file 
- **Videogame maps**: run readFromWarframe function in ```helper/ocupancyGridMap.py``` and import the return in the ambiente.py file 

To create your own maps:

- **2D indoor**: ```python3 helper/createScenario/createScenario.py``` and draw your scenario, when the pointer out from the screen the x and y nodes will be showed in the terminal.
- **2D/3D indoor**: the file ```helper/createScenario/caixas.py``` will export, in a .txt, all models to be used in a .world file (from gazebo) to build the environment. The sctructure is the same of 3D environment in ```3D/helper/ambiente.py```.
- **Forests**: the file ```helper/createScenario/createGazeboModel.py``` export a .world to be used in Gazebo simulator. The variable size1 and size2 define the size of the forest, sapce is the spacing between the trees, beta is the forest density. It is possible to add gaps with the function createClareira (example next to line 300). The alpha variable represents the gaps density. 

## Extra Features

To use travelling salesman problem use one of these codes:

```bash
  python3 tsp/cvDict.py
  python3 tsp/cvKNN.py
  python3 tsp/pcv.py
```

The first use a deterministic technique, the second use a K-Nearest Neighbor, and the last use a permutation.

To use coverage path planning with sweep algorithm:

```bash
  python3 coverageFolder/grid_based_sweep_coverage_path_planner.py
```

To use these algorithms in the primary code import them at the start and define the use. Commonly, the coverage algorithm is used in __ init __ and tsp is used to optimize some route.

## License

MIT License

Copyright (c) 2021 Lidia Gianne Souza da Rocha

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

  
## Feedback

If you have any feedback, please reach out to us at lidia@estudante.ufscar.br with subject "Plannie - Feedback".

  