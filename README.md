# AutonomousRobot
This project simulates our path-planning for an autonomous robot that tries to find a way to reach a goal (target)
in certainly environment 
### Usage:
``` python Robot_run.py -n <number of run times> -m <map name> -w <worldname> -r vision_range -sx <x> -sy <y> -gx <x> -gy <y> ```

* n: number of run times
    - < 0 or 0: run until meet the given goal
    - n: number of run times
    - default: 1
* m: input map name, default _map.csv
* w: input world model, no default.
* r: robot's vision range.
* sx, sy: start point of (x,y), type = float, default = 0.0, 0.0
* gx, gy: goal point of (x,y), type = float, default = 50.0, 50.0
* p: picking strategy
    - g: picking global first
    - l for picking local first, 
    - default: l (picking local first)
* rank_type: ranking type: ranking based on RRTtreeStar for or distance_and_angle formula
    - r: ranking by  RRTreeStar
    - da: ranking by distance_and_angle
    - default: r
### Examples:
##### Ranking by distance and angle for formula, picking_global_set_first strategy:
```
python Robot_run.py -n 5 -p g -rank_type da -m _MuchMoreFun.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0 -p
python Robot_run.py -n 0 -p g -rank_type da -m _map.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python Robot_run.py -n 0 -p g -rank_type da -w _world.png -sx 5 -sy 10 -gx 250 -gy 310 -r 40
```
##### Ranking by distance and angle for formula, picking_local_set_first strategy
```
python Robot_run.py -n 5 -p l -rank_type da -m _MuchMoreFun.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python Robot_run.py -n 0 -p l -rank_type da -m _map.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python Robot_run.py -n 0 -p l -rank_type da -w _world.png -sx 5 -sy 10 -gx 250 -gy 310 -r 40
```
##### Ranking by RRTreeStar, picking_local_set_first strategy
```
python Robot_run.py -n 5 -p l -rank_type r -m _MuchMoreFun.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python Robot_run.py -n 0 -p l -rank_type r -m _map.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python Robot_run.py -n 0 -p l -rank_type r -w _world.png -sx 5 -sy 10 -gx 250 -gy 310 -r 40
```
##### To run demo for RRTx algorithm:
```
python RRTree_X.py -m _map_blocks_1.csv
python RRTree_X.py -m _map_blocks_2.csv
python RRTree_X.py -n 5 -m _MuchMoreFun.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python RRTree_X.py -n 0 -m _map.csv -sx 5 -sy 5 -gx 35.0 -gy 50.0
python RRTree_X.py -n 0 -w _world.png -sx 5 -sy 10 -gx 250 -gy 310 -r 40
```
##### To see animation of RRT/RRTstart algorithms (without obstacle(s) ):
*   ss: sample size (default 2000)
```
python RRTree.py -ss 500
python RRTree_star.py -ss 500
```
For obstacle(s), i will update later if i have time

##### To run demo for the assumption of An and Hoai's Theory:
``` 
python Robot_theory.py -n 0 -r 100 -m _forest.csv -gx 500 -gy 500
python Robot_theory.py -n 0 -r  90 -m _forest.csv -gx 500 -gy 500
python Robot_theory.py -n 0 -r  80 -m _forest.csv -gx 500 -gy 500
```
* Set robot_vision parameter (option -r ) to see the different outcomes of experiments
##### To run experiment
``` 
python Easy_experiment.py -n 100 -m _forest.csv
python Easy_experiment.py -n 10 -w _world.png
```
recommend setting the following parameters in the program_config file to skip animation/printing.
```
easy_experiment = False
save_image = True
```

The results of experiment are:
* result<date_time>.csv file: contains all infomation in text form.
* pdf, images file: are plots at final step.
###### to visualize results of experiments:

```
python Easy_experiment_lib.py -r result<date_time>.csv
```

##### To generate a map:
Usage:

``` python map_generator.py -n <number of obstacles> -m <map name> -img <from_world_image>```

* mn: input map name, default _map_temp.csv
* n: number of obstacles.

Example 1 (generating map from user input):  

``` python map_generator.py -n 5 -m _map_temp.csv  ```
- Click on the given plot to input points
- Middle mouse click to turn next obstacle. Each obstacle contains a maximum of 100000 vertices

<img src="https://github.com/ThanhBinhTran/autonomousRobot/blob/main/Map_generator/map_display_user_input_demo.png" width="150" alt="world image">

Example 2 (generating map from image):

``` python map_generator.py -img _world.png ```

From world image <img src="https://github.com/ThanhBinhTran/autonomousRobot/blob/main/Map_generator/_world.png" width="150" alt="world image"> to map data (csv) <img src="https://github.com/ThanhBinhTran/autonomousRobot/blob/main/Map_generator/map_display_world_demo.png" width="150" alt="map data csv">



##### To display a map: 
``` python map_display.py -m <map name> ```

Example: ``` python map_display.py -m _map_temp.csv ```