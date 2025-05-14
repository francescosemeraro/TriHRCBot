# TriHRCBot: A Robotic Architecture for Triadic Human-Robot Collaboration through Mediated Object Alignment

For any questions and clarifications, please contact the developer Mr. Francesco Semeraro at:

francesco.semeraro@manchester.ac.uk

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

