## Setup instructions (for starting from scratch)
* If you don't already have the rest of this package on your machine,
  you can clone it from GitHub: [GlowingScrewdriver/eYRC-2023\_Cosmo\_Logistic-cl2703](https://github.com/GlowingScrewdriver/eYRC-2023_Cosmo_Logistic-cl2703)  
  _(you will need access to the repository; currently, only the team and user `eyantra` have access rights)_

* Build the package using colcon. For example:
  `$ colcon build --symlink-install --packages-select cl2703`

Note that this package, `cl2703`, is intended to be exposed as a Python module through Ament's
package index; as such, these scripts import each other through that
mechanism. For this reason, it is recommended that the entire package is built
and installed using colcon.


## Usage instructions
* Source the package's setup scripts:
  `$ . install/setup.sh` (from the same directory as the colcon build command)

* Make sure the controllers for eBot/Navigator2 and UR5/MoveIt are running.

* Turn off all magnets, start MoveIt servo and reset odometry and IMU:
  `$ ros2 run cl2703 utils.sh start_run`

* Place `config.yaml` in your current working directory.

* Run the task script:
  `$ ros2 run cl2703 task.py`


## Important files
* scripts/flags.py: flags to control behaviour of task scripts
* scripts/task*.py: scripts for the different eYRC CL tasks


_You can also find this file [online](https://github.com/GlowingScrewdriver/eYRC-2023_Cosmo_Logistic-cl2703/blob/main/README.md)_
