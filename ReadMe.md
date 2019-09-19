# ipa_rubbish_bin_detection

This package aims at providing the solution of rubbish bin detection. Currently, only rubbish bin with the cylinder shape is detected. Grasping points are selected in the top edge of the rubbish bin. 

Please change camera intrinsics and measure the height of the cylinder rubbish bin and insert it in the `launch/ipa_rubbish_bin_detection_params.yaml` file.

The cylinder rubbish bin information and grapsing points are published as `/cylinder_object` topic. 

Commands:

1. Launch the package: `roslaunch ipa_rubbish_bin_detection ipa_rubbish_bin_detection.launch`
2. Open `dynamic reconfigure` to change z_max_value to remove points in z direction. Some other parameters also can be changed in the `dynamic reconfigure` as well as `launch/ipa_rubbish_bin_detection_params.yaml` file (number of grapsing points, threshold values, ...)
3. Launch camera package (e.g realsense camera): `roslaunch realsense2_camera rs_rgbd.launch`. The defaut colored point cloud topic is `/camera/depth_registered/points`.

You can open RVIZ to check the result:

<img src="https://github.com/ipa-mah/ipa_rubbish_bin_detection/blob/master/etc/cylinder_detection.png" />


TODOs: 
 * Change the orientation of the grasping points.
 * Generalize shapes of rubbish bin.
 
Please contact me if you have any question.
