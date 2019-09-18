# ipa_rubbish_bin_detection

This package aims at providing the solution of rubish bin detection. Currently, only rubbish bin with the cylinder shape is detected. Graping points are selected in the top edge of the rubbish bin (TODO: change the orientation of the grasping points). 

Please change camera intrinsics and measure the height of the cylinder rubbish bin and insert it in the `launch/ipa_rubbish_bin_detection_params.yaml` file.

Commands:

1. Launch the package: roslaunch ipa_rubbish_bin_detection ipa_rubbish_bin_detection.launch 
2. Open `dynamic_reconfiguration` to change z_max_value to remove points in z direction. Some other parameters also can be changed in the `dynamic_reconfiguration` as well as `launch/ipa_rubbish_bin_detection_params.yaml` file.
3. Launch camera package (e.g realsense camera): roslaunch realsense2_camera rs_rgbd.launch. The defaut colored point cloud topic is `/camera/depth_registered/points`

You can open RVIZ to check the result.

