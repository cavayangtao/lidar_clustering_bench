# launch files

To run the clustering algorithm, corresponding parts need to be commented and uncommented in the launch files.

Take the EU long-term dataset as an example:
```
roslaunch package_name bag_play.launch bag:=bag_path
roslaunch package_name eult_clustering.launch
```

For the depth clustering only:
```
roslaunch package_name bag_play.launch  bag:=bag_path
rosrun depth_clustering show_objects_node --num_beams 32
```

