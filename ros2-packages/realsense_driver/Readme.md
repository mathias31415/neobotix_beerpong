# ROS2_Realsense_D415C

## Start docker container
```
./run_docker.sh
```
```
colcon build
```
```
source install/setup.bash
```


## Calibration
```
python calibration.py --dir TestbilderArucoCalibration/ --square_size 0.0212 --width 9 --height 7
```
Input:
 - Pictures of checkerboard patterns from different poses taken with your camera in folder `TestbilderArucoCalibration`
 - size of the square in metres (square_size)
 - shape of the checkerboard pattern (with, hight)

Output:
 - calibration_matrix.npy
 - distortion_coefficients.npy

calibration_matrix and distortion_coefficients are then used in the pose estimation of the aruco marker

## Aruco marker pose estimation 
Launch bringup:
```
ros2 launch neobotix_realsense415 realsense_bringup.launch.py
```

### Open second terminal:
```
docker exec -it <container_name> bash
```
```
source install/setup.bash
```

### Run application in second terminal --> Only test outputs on terminal:
```
ros2 run neobotix_realsense415 get_aruco_pose
```

### Start service in second terminal:
```
ros2 service call /get_aruco_pose camera_interfaces/srv/GetArucoPose "{aruco_id: <ID>}"
```
### Notes Aruco mit Calibrierung 0530

***alle in zero pose***
19: 
[{'id': 1, 'x': -0.2363647310296765, 'y': -0.17824478712922429, 'z': 0.7357854360610356, 'r': -2.1989203401350155, 'p': 2.293029403997893, 'j': 0.004704632578551419}]

38:
[{'id': 1, 'x': -0.20826846978831867, 'y': -0.17679704248702674, 'z': 0.7225905918908927, 'r': -2.2843426668282802, 'p': 2.2220924819412455, 'j': 0.005738637596565704}]

47:
[{'id': 1, 'x': -0.19913639563850297, 'y': -0.17430007789656973, 'z': 0.7045118577632048, 'r': -3.168218874736915, 'p': -0.01149650036245041, 'j': 0.09144157241312258}]

96:
[{'id': 1, 'x': -0.20409695667673586, 'y': -0.18345564544718995, 'z': 0.7096659443139286, 'r': -2.237441124170881, 'p': 2.254306227577299, 'j': 0.006968015442160814}]

***47mm tag in unterschiedlichen z-höhen***
``` uncalibrated (only intrinsics``` 
zero (0.4 to tcp):
[{'id': 1, 'x': -0.1993017634842499, 'y': -0.17411678258645305, 'z': 0.7037682207559435, 'r': 3.106681480533786, 'p': -0.040078438612544814, 'j': -0.08859214467127398}]

waypoint_1 (0.4 to tcp):
[{'id': 1, 'x': -0.09716446218490798, 'y': -0.17409337660227311, 'z': 0.6891468173188626, 'r': 3.069913976333951, 'p': -0.03112459696467047, 'j': -0.05471566236899043}]
--> x+0.1 

waypoint_3 (0.4 to tcp):
[{'id': 1, 'x': -0.1989309430995941, 'y': -0.07488623038031891, 'z': 0.7063588653574849, 'r': -3.155912648309544, 'p': 0.037550289966988976, 'j': 0.08072953473231864}]
--> y+0.1

waypoint_6 (0.5 to tcp):
[{'id': 1, 'x': -0.25165680606361057, 'y': -0.19382552475256326, 'z': 0.8629175663321048, 'r': -3.173177308490593, 'p': 0.03432491993461088, 'j': 0.11215839785002901}]
--> x-0.05, y-0.02, z+0.16

waypoint_10 (0.3 to tcp):
[{'id': 1, 'x': -0.14728817940881944, 'y': -0.15460845891535377, 'z': 0.5467565227510711, 'r': -3.1641538082619047, 'p': 0.021218557271449324, 'j': 0.09468825955031845}]
--> x+0.05, y+0.02, z-0.16

``` calibrated in plane xyz 0.4 --> try to calibrate linear z regression```

waypoint_14 (zero, but 0.6 to tcp):
[{'id': 1, 'x': -0.1028878752279313, 'y': -0.03229371827162553, 'z': 0.7144977876294323, 'r': -3.1725452414203357, 'p': 0.02282336733734962, 'j': 0.12120019865838226}]
--> x-0.1, y-0.03, z+0.11

waypoint_? (zero, but 0.55 to tcp):
[{'id': 1, 'x': -0.08074412507371187, 'y': -0.02648609177369654, 'z': 0.5595298548194573, 'r': None, 'p': None, 'j': None}]


waypoint_6 (zero, but 0.5 to tcp):
[{'id': 1, 'x': -0.04998117124563137, 'y': -0.013324324725076303, 'z': 0.5547790190884172, 'r': -3.1764621015961145, 'p': 0.03826291406782932, 'j': 0.1541329894734134}]
--> x-0.05, y-0.01, z+0.05

waypoint_x (zero, but 0.45 to tcp):
[{'id': 1, 'x': -0.027136154696552478, 'y': -0.0062820838836109005, 'z': 0.4867773080800371, 'r': -3.1721548142824556, 'p': 0.03071845590690283, 'j': 0.10438153790111022}]
-->x-0.027, y-0.006, z+0.036

zero (0.4 to tcp):
[{'id': 1, 'x': 0.0012627399148642249, 'y': 0.005267152660585056, 'z': 0.40000721037719217, 'r': -3.195506042311811, 'p': 0.03998766325012742, 'j': 0.08842309634495739}]
-->x:0, y:0, z:0

waypoint_x (zero, but 0.35 to tcp):
[{'id': 1, 'x': 0.02495384184038213, 'y': 0.013122530566999996, 'z': 0.32953580624601125, 'r': -3.1684891032543128, 'p': 0.02349728664782181, 'j': 0.08385941337999202}]
-->x:0.025, y:0.013, z:0.02

waypoint_10 (zero, but 0.3 to tcp):
[{'id': 1, 'x': 0.0516477851543565, 'y': 0.023355616238775245, 'z': 0.24811140540941307, 'r': -3.183125065295335, 'p': 0.031079961396930016, 'j': 0.09125738581595105}]
--> x+0.05 ,y+0.023, z-0.05

waypoint_? (zero, but 0.25 to tcp):
[{'id': 1, 'x': 0.07651819667374102, 'y': 0.032457009110609336, 'z': 0.17222668848953399, 'r': -3.1670694779284077, 'p': 0.032724530508340774, 'j': 0.0988606082536651}]
--> x+0.076, y+0.032, z-0.075

------------
waypoint_1:
[{'id': 1, 'x': 0.10357729524857721, 'y': 0.00632238414838146, 'z': 0.38055888791999554, 'r': 3.009197970622601, 'p': -0.044849467747631856, 'j': -0.037322317128087014}]
--> x+0.1

waypoint_3:
[{'id': 1, 'x': 6.905690040590051e-05, 'y': 0.10331376961968108, 'z': 0.40745886535748493, 'r': -3.155912648309544, 'p': 0.037550289966988976, 'j': 0.08072953473231864}]
--> y+0.1

waypoint_9 (0.5 to tcp and x+0.1):
[{'id': 1, 'x': 0.0524267632506428, 'y': -0.012912089273074662, 'z': 0.5370305817016139, 'r': -3.2392049167447, 'p': 0.039570524485000196, 'j': 0.12046219781880146}]
--> x-0.05, y-0.01, z+0.04
----------

``` after lookup in z implemented```

*** test z lookup ***
waypoint_14 (zero, but 0.6 to tcp):
[{'id': 1, 'x': -0.10638298604332447, 'y': -0.03536587867399066, 'z': 0.6068485400332908, 'r': None, 'p': None, 'j': None}]

waypoint_6 (zero, but 0.5 to tcp):
[{'id': 1, 'x': -0.051144124925694257, 'y': -0.014140295673787967, 'z': 0.5016533478265855, 'r': None, 'p': None, 'j': None}]

TEST (zero, but 0.55 to tcp):
[{'id': 1, 'x': -0.07825579375124919, 'y': -0.024504247300263376, 'z': 0.5537203578033733, 'r': None, 'p': None, 'j': None}]

zero (0.4 to tcp:):
[{'id': 1, 'x': -0.0004643906442921564, 'y': 0.004123519185701613, 'z': 0.40272020062750136, 'r': None, 'p': None, 'j': None}]

waypoint_10 (zero, but 0.3 to tcp):
[{'id': 1, 'x': 0.050362655518487426, 'y': 0.022794008406057303, 'z': 0.30191553691826556, 'r': None, 'p': None, 'j': None}]



*** capture new lookup data for x calibration ***
waypoint_? (0.25 to tcp):
[{'id': 1, 'x': 0.1754993348051619, 'y': 0.028778149045498608, 'z': 0.2516524179135027, 'r': None, 'p': None, 'j': None}]
--> x-0.075

waypoint_17 (0.3 to tcp):
[{'id': 1, 'x': 0.15108675331684138, 'y': 0.020846336635461482, 'z': 0.2984821832634882, 'r': None, 'p': None, 'j': None}]
--> x-0.05

waypoint_?? (0.35 to tcp):
[{'id': 1, 'x': 0.12653238116696566, 'y': 0.013232078018893173, 'z': 0.34182673915088224, 'r': None, 'p': None, 'j': None}]
--> x-0.026

waypoint_1 (0.4 to tcp):
[{'id': 1, 'x': 0.10183553781509203, 'y': 0.004106623397726883, 'z': 0.39308284916231395, 'r': None, 'p': None, 'j': None}]
--> 0

waypoint_?? (0.45 to tcp):
[{'id': 1, 'x': 0.07584635677775599, 'y': -0.00572372465292495, 'z': 0.44099182848230944, 'r': None, 'p': None, 'j': None}]
--> x+0.025

waypoint_9 (0.5 to tcp):
[{'id': 1, 'x': 0.05068810279452074, 'y': -0.014612991443792689, 'z': 0.49334842701038867, 'r': None, 'p': None, 'j': None}]
--> x+0.05

waypoint_? (0.55 to tcp):
[{'id': 1, 'x': 0.02488358208811753, 'y': -0.02416577211737439, 'z': 0.543749145016624, 'r': None, 'p': None, 'j': None}]
--> x+0.075

waypoint_17 (0.6 to tcp):
[{'id': 1, 'x': -0.0025083978818770114, 'y': -0.03590782776233045, 'z': 0.5985013649498825, 'r': None, 'p': None, 'j': None}]
--> x+0.1

--> done !

*** capture new lookup data for y calibration ***

--> y +0.1
0.25 to tcp:
[{'id': 1, 'x': 0.0028068662937209, 'y': 0.13061533439699918, 'z': 0.2527063304549692, 'r': None, 'p': None, 'j': None}]
--> y-0.031

0.3 to tcp:
[{'id': 1, 'x': 0.002764616999251124, 'y': 0.12120175892948176, 'z': 0.305225674405124, 'r': None, 'p': None, 'j': None}]
--> y-0.021

0.35 to tcp:
[{'id': 1, 'x': 0.0014816937331228706, 'y': 0.11148981479136429, 'z': 0.35510218583635444, 'r': None, 'p': None, 'j': None}]
--> y-0.011

waypoint_3 (0.4 to tcp):
[{'id': 1, 'x': 0.002002021013280786, 'y': 0.10368382486029262, 'z': 0.401248049674457, 'r': None, 'p': None, 'j': None}]
--> y 0

0.45 to tcp:
[{'id': 1, 'x': -0.0012637012916743434, 'y': 0.09471153451236959, 'z': 0.44553977591985344, 'r': None, 'p': None, 'j': None}]
--> y+0.005

0.5 to tcp:
[{'id': 1, 'x': 0.0009250608285241366, 'y': 0.08589247680451031, 'z': 0.501270781897492, 'r': None, 'p': None, 'j': None}]
--> y+0.015

0.55 to tcp:
[{'id': 1, 'x': -0.0005868872394499958, 'y': 0.07671416797810131, 'z': 0.5423340531902135, 'r': None, 'p': None, 'j': None}]
--> y+0.023

0.6 to tcp:
[{'id': 1, 'x': -0.0017714097045336274, 'y': 0.06780039128423122, 'z': 0.6003879785279712, 'r': None, 'p': None, 'j': None}]
--> y+0.032


--> y -0.1
0.5 to tcp:
[{'id': 1, 'x': -0.0008334092395981402, 'y': -0.11443696874968842, 'z': 0.5025623086702444, 'r': None, 'p': None, 'j': None}]
-->y+0.015


--> y+0.2
0.3 to tcp:
[{'id': 1, 'x': 0.003949227052040705, 'y': 0.22041783592253347, 'z': 0.2938531586191888, 'r': None, 'p': None, 'j': None}]

0.5 to tcp:
[{'id': 1, 'x': 0.0018345734200185676, 'y': 0.18501166626102958, 'z': 0.4910643299271927, 'r': None, 'p': None, 'j': None}]











