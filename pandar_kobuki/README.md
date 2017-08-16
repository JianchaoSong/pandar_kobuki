# pandar_kobuki的说明

pandar_kobuki是完成探索和导航的文件包。在这个包集里，有kobuki_description、turtlebot_description-的改动编辑，用来做可视化的显示。

## 探索任务
	
	采用gmapping完成建图，并在这个过程中采用move_base完成移动机器人的自主控制。

	```
		roslaunch pcl_bag pandarExploration.launch
	````

## 导航任务

	相对于探索导航的地图是可以提前完成构建的，

	```
		roslaunch pcl_bag pandarNav.alunch map_dir:="the path of map you want use .yaml"
	```

