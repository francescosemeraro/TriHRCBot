# TriHRCBot: A Robotic Architecture for Triadic Human-Robot Collaboration through Mediated Object Alignment

For any questions and clarifications, please contact the developer Dr Francesco Semeraro at:

francesco.semeraro@iit.it

Read full paper <a href="https://ieeexplore.ieee.org/abstract/document/11128795">here</a>, Presented at ICRA 2025.

TriHRCBot is a robotic architecture that enables a robotic manipulator to autonomously handle position and orientationof a shared target between two users who have to perform different operations each on different faces of the object. The architecture takes into account position, orientation, bodily lengths and ongoing state of the interaction to update the pose of the object. The architecture was designed so that it can be used on any collaborative task of the described kind, without he nee of retraining the architecture.

### How to cite our paper
If any of the content of this repository was helpful to you, please cite our paper using:
```
@INPROCEEDINGS{11128795,
  author={Semeraro, Francesco and Leadbetter, James and Cangelosi, Angelo},
  booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={TriHRCBot: A Robotic Architecture for Triadic Human-Robot Collaboration Through Mediated Object Alignment}, 
  year={2025},
  volume={},
  number={},
  pages={6703-6709},
  keywords={Hands;Deep learning;Codes;Autonomous systems;Collaboration;Human-robot interaction;Robot sensing systems},
  doi={10.1109/ICRA55743.2025.11128795}}

```

### Requirements
To be able to run all the scripts, you will need to install the following:
<br />
1- Python 3.10 <br />
2- ROS 2 Humble Hawksbill  <br />
3- sqlite3. <br />
<br />

### The repository
The main elements of the repository are the following:
<br />
1- "robotic_system", C++ package that contains the Trajectory Planner node and other low-level routines<br />
2- "non_dyadic_ai", Python package that contains the Perception layer, Trajectory Planner and Object Pose Aligner nodes <br />
3- "netwroks", folder containing the deep neural network used in the Joint Activity classifier node  <br />
4- "interface_msgs", custom ROS 2 messages for this workspace.  <br />
5- "my_ur_launch", package become obsolete, soon to be removed  <br />
<br />

