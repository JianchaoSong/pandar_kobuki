#pcl-bag的使用说明
pcl-bag的任务是一堆小程序，每个完成简单的任务，例如数据的转换
pcl-bag_node的任务是将一个pcl类型文件的文件夹呢按照格式定义的pcl转换为ros的pointcloud2类型
##pcl-bag_node具体说明
usage:::::::: of programe
rosrun pcl_bag pcl-bag_node path_to_pcd_dir
like this:

rosrun pcl_bag pcl-bag_node /home/jc/pcd_0727

in dir the file name just like pcl_$count_measure.pcd

##pclSurf_node具体说明
这个文件是测试，测试如何将建好的点云提取面特征

##pandarTo2D具体说明
订阅pandar_node/pandar_points发布laserscan类型的数据

##pclColor具体说明
将已经slam建好的pcl图，根据位置结构，如z的大小完成不同区域的润色

