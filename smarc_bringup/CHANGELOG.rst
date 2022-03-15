^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package smarc_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Changed name of pkg containing lolo
* The repo is now BSD-3 licensed
* Merge pull request `#17 <https://github.com/smarc-project/smarc_utils/issues/17>`_ from ignaciotb/before_dryrun
  Before dryrun
* Added node to clean up Gazebo
* No teleop or navigation
* Merge pull request `#15 <https://github.com/smarc-project/smarc_utils/issues/15>`_ from nilsbore/new_master
  No teleop or navigation
* Merge pull request `#14 <https://github.com/smarc-project/smarc_utils/issues/14>`_ from ignaciotb/master
  Merged pipefollower demo with master branch
* A launch file has been created in smarc_scenarios for the pipe_following demo
* Added rock mesh as a param
* Pipe follower demo. Motala
* Update README.md
* Update README.md
* Merge pull request `#12 <https://github.com/smarc-project/smarc_utils/issues/12>`_ from ignaciotb/master
  Added README with some user instructions. Fantastic, good work!
* Added README with some user instructions
* Merge pull request `#8 <https://github.com/smarc-project/smarc_utils/issues/8>`_ from ignaciotb/master
  Renaming of a some nodes + changes to instantiate several AUVs
* Added general launch file call for planning components
* Fixed passing of args through launch files
* Added auv_system.launch as top level launch file for AUV instances
* Removed unused vars from auv_scenarios.launch
* Final launch files hierarchy working
* Second version of auv_model.launch
* Splitted bring_up launch file into scenario and auv
* Renamed smarc_lm_visualizer pkg
* Merge pull request `#7 <https://github.com/smarc-project/smarc_utils/issues/7>`_ from ignaciotb/master
  odom_tf_bc will bc world-->map now
* Added map frame provider. Same pose than odom frame
* Merge pull request `#5 <https://github.com/smarc-project/smarc_utils/issues/5>`_ from ignaciotb/master
  Added node to publish rocks as landmarks in RVIZ. Looking good, merging!
* Added landmarks visualizer node to auv_scenarios.launch
* Merge pull request `#3 <https://github.com/smarc-project/smarc_utils/issues/3>`_ from ignaciotb/working_branch
  Corrected params in auv_scenarios.launch. Looking good!
* Corrected params in auv_scenarios.launch
* Merge pull request `#2 <https://github.com/smarc-project/smarc_utils/issues/2>`_ from ignaciotb/working_branch
  Working branch. Looking good.
* changed auv_name for namespace
* Merge pull request `#1 <https://github.com/smarc-project/smarc_utils/issues/1>`_ from ignaciotb/working_branch
  Working branch. Good job!
* Removed rviz folder from smarc_bringup
* Removing RVIZ from top-level launch file
* Added docs folder for documenting the README
* Added .rviz file with basics
* Deleted old launch for pipe following scenario
* Launch files and args renamed for generalization of their use
* Now odom_tf_bc broadcasts odom tf if launch flag run_navigation is set to false
* The odom_tf_bc node has been moved to the upper level launch file!
* ekf_random_rocks launch file
* Added launch file for ekf tests
* Moved localization components to upload_lolo_auv.launch
* Created tf publisher world-frame
* Merge branch 'master' of https://github.com/smarc-project/smarc_utils
* Created a pipe_following launch and rviz files for quick testing
* Changed the pipe following script also
* Created lolo_pipe_following.launch for simplicity
* Changed the bringup script to new version of pipe following
* Set the starting position
* Fixed package name
* Added a bringup script for pipe following
* Contributors: Nacho, Nils Bore
