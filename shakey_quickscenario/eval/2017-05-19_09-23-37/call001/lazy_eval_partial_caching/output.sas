begin_variables
108
var0 2 -1
var1 2 -1
var2 2 -1
var3 2 -1
var4 2 -1
var5 2 -1
var6 2 -1
var7 2 -1
var8 2 -1
var9 2 -1
var10 2 -1
var11 2 -1
var12 2 -1
var13 2 -1
var14 2 -1
var15 2 -1
var16 2 -1
var17 2 -1
var18 2 -1
var19 2 -1
var20 2 -1
var21 2 -1
var22 2 -1
var23 2 -1
var24 2 -1
var25 2 -1
var26 2 -1
var27 2 -1
var28 2 -1
var29 2 -1
var30 2 -1
var31 2 -1
var32 2 -1
var33 2 -1
var34 2 -1
var35 2 -1
var36 2 -1
var37 2 -1
var38 2 -1
var39 2 -1
var40 2 -1
var41 2 -1
var42 2 -1
var43 2 -1
var44 2 -1
var45 2 -1
var46 2 -1
var47 2 -1
var48 2 -1
var49 2 -1
var50 2 -1
var51 2 -1
var52 2 -1
var53 2 -1
var54 2 -1
var55 2 -1
var56 2 -1
var57 2 -1
var58 2 -1
var59 2 -1
var60 2 -1
var61 2 -1
var62 2 -1
var63 2 -1
var64 2 -1
var65 2 -1
var66 2 -1
var67 2 -1
var68 2 -1
var69 2 -1
var70 2 -1
var71 2 -1
var72 2 -1
var73 2 -1
var74 2 -1
var75 2 -1
var76 2 -1
var77 2 -1
var78 2 -1
var79 2 -1
var80 2 -1
var81 2 -1
var82 2 -1
var83 2 -1
var84 2 2
var85 2 -1
var86 2 1
var87 2 -1
var88 2 -1
var89 2 -1
var90 2 -1
var91 -1 -1
var92 -1 -1
var93 -1 -1
var94 -2 -1
var95 -2 -1
var96 -2 -1
var97 -2 -1
var98 -2 -1
var99 -2 -1
var100 -2 -1
var101 -2 -1
var102 -2 -1
var103 -2 -1
var104 -2 -1
var105 -2 -1
var106 -2 -1
var107 -2 -1
end_variables
begin_oplinits
0
end_oplinits
begin_objects
21
location robot_location
doorway doorway0
doorway doorway1
doorway_in_location doorway0_room0
doorway_in_location doorway0_room1
doorway_in_location doorway1_room1
doorway_in_location doorway1_room2
doorway_out_location doorway0_room0_out
doorway_out_location doorway0_room1_out
doorway_out_location doorway1_room1_out
doorway_out_location doorway1_room2_out
frameid /map
location init_loc
location robot_location
object_location object_location_loc0_room0
object_location object_location_loc1_room2
room room0
room room1
room room2
search_location search_location_loc0_room2
search_location search_location_loc1_room0
end_objects
begin_pddl_translation
88
at-base 1 doorway0_room0 0 0
at-base 1 doorway0_room0_out 1 0
at-base 1 doorway0_room1 2 0
at-base 1 doorway0_room1_out 3 0
at-base 1 doorway1_room1 4 0
at-base 1 doorway1_room1_out 5 0
at-base 1 doorway1_room2 6 0
at-base 1 doorway1_room2_out 7 0
at-base 1 init_loc 8 0
at-base 1 object_location_loc0_room0 9 0
at-base 1 object_location_loc1_room2 10 0
at-base 1 robot_location 11 0
at-base 1 search_location_loc0_room2 12 0
at-base 1 search_location_loc1_room0 13 0
can-navigate 2 doorway0_room0 doorway0_room0 14 0
can-navigate 2 doorway0_room0 doorway0_room0_out 15 0
can-navigate 2 doorway0_room0 init_loc 16 0
can-navigate 2 doorway0_room0 object_location_loc0_room0 17 0
can-navigate 2 doorway0_room0 robot_location 18 0
can-navigate 2 doorway0_room0 search_location_loc1_room0 19 0
can-navigate 2 doorway0_room0_out doorway0_room0 20 0
can-navigate 2 doorway0_room0_out doorway0_room0_out 21 0
can-navigate 2 doorway0_room0_out init_loc 22 0
can-navigate 2 doorway0_room0_out object_location_loc0_room0 23 0
can-navigate 2 doorway0_room0_out robot_location 24 0
can-navigate 2 doorway0_room0_out search_location_loc1_room0 25 0
can-navigate 2 doorway0_room1 doorway0_room1 26 0
can-navigate 2 doorway0_room1 doorway0_room1_out 27 0
can-navigate 2 doorway0_room1 doorway1_room1 28 0
can-navigate 2 doorway0_room1 doorway1_room1_out 29 0
can-navigate 2 doorway0_room1_out doorway0_room1 30 0
can-navigate 2 doorway0_room1_out doorway0_room1_out 31 0
can-navigate 2 doorway0_room1_out doorway1_room1 32 0
can-navigate 2 doorway0_room1_out doorway1_room1_out 33 0
can-navigate 2 doorway1_room1 doorway0_room1 34 0
can-navigate 2 doorway1_room1 doorway0_room1_out 35 0
can-navigate 2 doorway1_room1 doorway1_room1 36 0
can-navigate 2 doorway1_room1 doorway1_room1_out 37 0
can-navigate 2 doorway1_room1_out doorway0_room1 38 0
can-navigate 2 doorway1_room1_out doorway0_room1_out 39 0
can-navigate 2 doorway1_room1_out doorway1_room1 40 0
can-navigate 2 doorway1_room1_out doorway1_room1_out 41 0
can-navigate 2 doorway1_room2 doorway1_room2 42 0
can-navigate 2 doorway1_room2 doorway1_room2_out 43 0
can-navigate 2 doorway1_room2 object_location_loc1_room2 44 0
can-navigate 2 doorway1_room2 search_location_loc0_room2 45 0
can-navigate 2 doorway1_room2_out doorway1_room2 46 0
can-navigate 2 doorway1_room2_out doorway1_room2_out 47 0
can-navigate 2 doorway1_room2_out object_location_loc1_room2 48 0
can-navigate 2 doorway1_room2_out search_location_loc0_room2 49 0
can-navigate 2 init_loc doorway0_room0 50 0
can-navigate 2 init_loc doorway0_room0_out 51 0
can-navigate 2 init_loc init_loc 52 0
can-navigate 2 init_loc object_location_loc0_room0 53 0
can-navigate 2 init_loc robot_location 54 0
can-navigate 2 init_loc search_location_loc1_room0 55 0
can-navigate 2 object_location_loc0_room0 doorway0_room0 56 0
can-navigate 2 object_location_loc0_room0 doorway0_room0_out 57 0
can-navigate 2 object_location_loc0_room0 init_loc 58 0
can-navigate 2 object_location_loc0_room0 object_location_loc0_room0 59 0
can-navigate 2 object_location_loc0_room0 robot_location 60 0
can-navigate 2 object_location_loc0_room0 search_location_loc1_room0 61 0
can-navigate 2 object_location_loc1_room2 doorway1_room2 62 0
can-navigate 2 object_location_loc1_room2 doorway1_room2_out 63 0
can-navigate 2 object_location_loc1_room2 object_location_loc1_room2 64 0
can-navigate 2 object_location_loc1_room2 search_location_loc0_room2 65 0
can-navigate 2 robot_location doorway0_room0 66 0
can-navigate 2 robot_location doorway0_room0_out 67 0
can-navigate 2 robot_location init_loc 68 0
can-navigate 2 robot_location object_location_loc0_room0 69 0
can-navigate 2 robot_location robot_location 70 0
can-navigate 2 robot_location search_location_loc1_room0 71 0
can-navigate 2 search_location_loc0_room2 doorway1_room2 72 0
can-navigate 2 search_location_loc0_room2 doorway1_room2_out 73 0
can-navigate 2 search_location_loc0_room2 object_location_loc1_room2 74 0
can-navigate 2 search_location_loc0_room2 search_location_loc0_room2 75 0
can-navigate 2 search_location_loc1_room0 doorway0_room0 76 0
can-navigate 2 search_location_loc1_room0 doorway0_room0_out 77 0
can-navigate 2 search_location_loc1_room0 init_loc 78 0
can-navigate 2 search_location_loc1_room0 object_location_loc0_room0 79 0
can-navigate 2 search_location_loc1_room0 robot_location 80 0
can-navigate 2 search_location_loc1_room0 search_location_loc1_room0 81 0
doorway-state-known 1 doorway0 82 0
doorway-state-known 1 doorway1 83 0
object-at-location 1 object_location_loc0_room0 87 0
object-at-location 1 object_location_loc1_room2 88 0
searched 1 search_location_loc0_room2 89 0
searched 1 search_location_loc1_room0 90 0
0
end_pddl_translation
begin_constant_facts
36
belongs-to-doorway!val 2 doorway0_room0 doorway0
belongs-to-doorway!val 2 doorway0_room0_out doorway0
belongs-to-doorway!val 2 doorway0_room1 doorway0
belongs-to-doorway!val 2 doorway0_room1_out doorway0
belongs-to-doorway!val 2 doorway1_room1 doorway1
belongs-to-doorway!val 2 doorway1_room1_out doorway1
belongs-to-doorway!val 2 doorway1_room2 doorway1
belongs-to-doorway!val 2 doorway1_room2_out doorway1
frame-id!val 2 doorway0_room0 /map
frame-id!val 2 doorway0_room0_out /map
frame-id!val 2 doorway0_room1 /map
frame-id!val 2 doorway0_room1_out /map
frame-id!val 2 doorway1_room1 /map
frame-id!val 2 doorway1_room1_out /map
frame-id!val 2 doorway1_room2 /map
frame-id!val 2 doorway1_room2_out /map
frame-id!val 2 init_loc /map
frame-id!val 2 object_location_loc0_room0 /map
frame-id!val 2 object_location_loc1_room2 /map
frame-id!val 2 robot_location /map
frame-id!val 2 search_location_loc0_room2 /map
frame-id!val 2 search_location_loc1_room0 /map
location-in-room!val 2 doorway0_room0 room0
location-in-room!val 2 doorway0_room0_out room0
location-in-room!val 2 doorway0_room1 room1
location-in-room!val 2 doorway0_room1_out room1
location-in-room!val 2 doorway1_room1 room1
location-in-room!val 2 doorway1_room1_out room1
location-in-room!val 2 doorway1_room2 room2
location-in-room!val 2 doorway1_room2_out room2
location-in-room!val 2 init_loc room0
location-in-room!val 2 object_location_loc0_room0 room0
location-in-room!val 2 object_location_loc1_room2 room2
location-in-room!val 2 robot_location room0
location-in-room!val 2 search_location_loc0_room2 room2
location-in-room!val 2 search_location_loc1_room0 room0
112
qw 1 doorway0_room0 0.999801
qw 1 doorway0_room0_out -0.019957
qw 1 doorway0_room1 -0.019813
qw 1 doorway0_room1_out -0.999804
qw 1 doorway1_room1 0.999967
qw 1 doorway1_room1_out -0.008175
qw 1 doorway1_room2 0.012267
qw 1 doorway1_room2_out -0.999925
qw 1 init_loc 1.000000
qw 1 object_location_loc0_room0 1.000000
qw 1 object_location_loc1_room2 1.000000
qw 1 robot_location 1.000000
qw 1 search_location_loc0_room2 -0.179464
qw 1 search_location_loc1_room0 0.902964
qx 1 doorway0_room0 0.000000
qx 1 doorway0_room0_out 0.000000
qx 1 doorway0_room1 0.000000
qx 1 doorway0_room1_out 0.000000
qx 1 doorway1_room1 0.000000
qx 1 doorway1_room1_out 0.000000
qx 1 doorway1_room2 0.000000
qx 1 doorway1_room2_out 0.000000
qx 1 init_loc 0.000000
qx 1 object_location_loc0_room0 0.000000
qx 1 object_location_loc1_room2 0.000000
qx 1 robot_location -0.000000
qx 1 search_location_loc0_room2 0.000000
qx 1 search_location_loc1_room0 0.000000
qy 1 doorway0_room0 0.000000
qy 1 doorway0_room0_out 0.000000
qy 1 doorway0_room1 0.000000
qy 1 doorway0_room1_out 0.000000
qy 1 doorway1_room1 0.000000
qy 1 doorway1_room1_out 0.000000
qy 1 doorway1_room2 0.000000
qy 1 doorway1_room2_out 0.000000
qy 1 init_loc 0.000000
qy 1 object_location_loc0_room0 0.000000
qy 1 object_location_loc1_room2 0.000000
qy 1 robot_location -0.000000
qy 1 search_location_loc0_room2 0.000000
qy 1 search_location_loc1_room0 0.000000
qz 1 doorway0_room0 0.019957
qz 1 doorway0_room0_out 0.999801
qz 1 doorway0_room1 0.999804
qz 1 doorway0_room1_out -0.019813
qz 1 doorway1_room1 0.008175
qz 1 doorway1_room1_out 0.999967
qz 1 doorway1_room2 0.999925
qz 1 doorway1_room2_out 0.012267
qz 1 init_loc 0.000000
qz 1 object_location_loc0_room0 0.000000
qz 1 object_location_loc1_room2 0.000000
qz 1 robot_location 0.000085
qz 1 search_location_loc0_room2 0.983765
qz 1 search_location_loc1_room0 -0.429715
timestamp 1 doorway0_room0 1625.210000
timestamp 1 doorway0_room0_out 1625.210000
timestamp 1 doorway0_room1 1690.030000
timestamp 1 doorway0_room1_out 1690.030000
timestamp 1 doorway1_room1 1756.040000
timestamp 1 doorway1_room1_out 1756.040000
timestamp 1 doorway1_room2 1821.770000
timestamp 1 doorway1_room2_out 1821.770000
timestamp 1 init_loc 446.568000
timestamp 1 object_location_loc0_room0 1366380000.000000
timestamp 1 object_location_loc1_room2 1366380000.000000
timestamp 1 robot_location 457.970000
timestamp 1 search_location_loc0_room2 1495180000.000000
timestamp 1 search_location_loc1_room0 1495170000.000000
x 1 doorway0_room0 3.455620
x 1 doorway0_room0_out 3.455620
x 1 doorway0_room1 8.200000
x 1 doorway0_room1_out 8.200000
x 1 doorway1_room1 7.035100
x 1 doorway1_room1_out 7.035100
x 1 doorway1_room2 11.653700
x 1 doorway1_room2_out 11.653700
x 1 init_loc 0.000000
x 1 object_location_loc0_room0 -5.000000
x 1 object_location_loc1_room2 12.000000
x 1 robot_location 0.012766
x 1 search_location_loc0_room2 17.362400
x 1 search_location_loc1_room0 -4.875510
y 1 doorway0_room0 1.481760
y 1 doorway0_room0_out 1.481760
y 1 doorway0_room1 1.505140
y 1 doorway0_room1_out 1.505140
y 1 doorway1_room1 -3.551470
y 1 doorway1_room1_out -3.551470
y 1 doorway1_room2 -3.527860
y 1 doorway1_room2_out -3.527860
y 1 init_loc 0.000000
y 1 object_location_loc0_room0 -1.000000
y 1 object_location_loc1_room2 3.500000
y 1 robot_location -0.001858
y 1 search_location_loc0_room2 0.833664
y 1 search_location_loc1_room0 3.263760
z 1 doorway0_room0 -0.000092
z 1 doorway0_room0_out -0.000092
z 1 doorway0_room1 -0.000000
z 1 doorway0_room1_out -0.000000
z 1 doorway1_room1 0.000000
z 1 doorway1_room1_out 0.000000
z 1 doorway1_room2 0.000000
z 1 doorway1_room2_out 0.000000
z 1 init_loc 0.000000
z 1 object_location_loc0_room0 0.000000
z 1 object_location_loc1_room2 0.000000
z 1 robot_location 0.051000
z 1 search_location_loc0_room2 0.000000
z 1 search_location_loc1_room0 0.000000
end_constant_facts
begin_modules
0
0
0
14
canDriveToPos@libshakey_planning_actions.so 1 ?g location doorway0_room0 94
canDriveToPos@libshakey_planning_actions.so 1 ?g location doorway0_room0_out 95
canDriveToPos@libshakey_planning_actions.so 1 ?g location doorway0_room1 96
canDriveToPos@libshakey_planning_actions.so 1 ?g location doorway0_room1_out 97
canDriveToPos@libshakey_planning_actions.so 1 ?g location doorway1_room1 98
canDriveToPos@libshakey_planning_actions.so 1 ?g location doorway1_room1_out 99
canDriveToPos@libshakey_planning_actions.so 1 ?g location doorway1_room2 100
canDriveToPos@libshakey_planning_actions.so 1 ?g location doorway1_room2_out 101
canDriveToPos@libshakey_planning_actions.so 1 ?g location init_loc 102
canDriveToPos@libshakey_planning_actions.so 1 ?g location object_location_loc0_room0 103
canDriveToPos@libshakey_planning_actions.so 1 ?g location object_location_loc1_room2 104
canDriveToPos@libshakey_planning_actions.so 1 ?g location robot_location 105
canDriveToPos@libshakey_planning_actions.so 1 ?g location search_location_loc0_room2 106
canDriveToPos@libshakey_planning_actions.so 1 ?g location search_location_loc1_room0 107
0
0
0
end_modules
begin_state
1
1
1
1
1
1
1
1
0
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
1
0
1
1
1
1
1.0
10.0
1000.0
-3
-3
-3
-3
-3
-3
-3
-3
-3
-3
-3
-3
-3
-3
end_state
begin_goal
1
84 0
end_goal
64
begin_operator
detect-doorway-state doorway0_room0 doorway0
0
= 91
2
0 0
82 1
0
0
0
1
0 0 0 82 -1 0
end_operator
begin_operator
detect-doorway-state doorway0_room1 doorway0
0
= 91
2
2 0
82 1
0
0
0
1
0 0 0 82 -1 0
end_operator
begin_operator
detect-doorway-state doorway1_room1 doorway1
0
= 91
2
83 1
4 0
0
0
0
1
0 0 0 83 -1 0
end_operator
begin_operator
detect-doorway-state doorway1_room2 doorway1
0
= 91
2
83 1
6 0
0
0
0
1
0 0 0 83 -1 0
end_operator
begin_operator
detect-objects search_location_loc0_room2
0
= 91
2
89 1
12 0
0
0
0
1
0 0 0 89 -1 0
end_operator
begin_operator
detect-objects search_location_loc1_room0
0
= 91
2
90 1
13 0
0
0
0
1
0 0 0 90 -1 0
end_operator
begin_operator
drive-base doorway0_room0 doorway0_room0_out
0
= 93
1
95 0
0
0
1
0 0 0 0 0 1
1
0 0 0 1 -1 0
end_operator
begin_operator
drive-base doorway0_room0 init_loc
0
= 93
1
102 0
0
0
1
0 0 0 0 0 1
1
0 0 0 8 -1 0
end_operator
begin_operator
drive-base doorway0_room0 object_location_loc0_room0
0
= 93
1
103 0
0
0
1
0 0 0 0 0 1
1
0 0 0 9 -1 0
end_operator
begin_operator
drive-base doorway0_room0 robot_location
0
= 93
2
105 0
18 0
0
0
1
0 0 0 0 0 1
1
0 0 0 11 -1 0
end_operator
begin_operator
drive-base doorway0_room0 search_location_loc1_room0
0
= 93
1
107 0
0
0
1
0 0 0 0 0 1
1
0 0 0 13 -1 0
end_operator
begin_operator
drive-base doorway0_room0_out doorway0_room0
0
= 93
1
94 0
0
0
1
0 0 0 1 0 1
1
0 0 0 0 -1 0
end_operator
begin_operator
drive-base doorway0_room0_out init_loc
0
= 93
1
102 0
0
0
1
0 0 0 1 0 1
1
0 0 0 8 -1 0
end_operator
begin_operator
drive-base doorway0_room0_out object_location_loc0_room0
0
= 93
1
103 0
0
0
1
0 0 0 1 0 1
1
0 0 0 9 -1 0
end_operator
begin_operator
drive-base doorway0_room0_out robot_location
0
= 93
2
24 0
105 0
0
0
1
0 0 0 1 0 1
1
0 0 0 11 -1 0
end_operator
begin_operator
drive-base doorway0_room0_out search_location_loc1_room0
0
= 93
1
107 0
0
0
1
0 0 0 1 0 1
1
0 0 0 13 -1 0
end_operator
begin_operator
drive-base doorway0_room1 doorway0_room1_out
0
= 93
1
97 0
0
0
1
0 0 0 2 0 1
1
0 0 0 3 -1 0
end_operator
begin_operator
drive-base doorway0_room1 doorway1_room1
0
= 93
1
98 0
0
0
1
0 0 0 2 0 1
1
0 0 0 4 -1 0
end_operator
begin_operator
drive-base doorway0_room1 doorway1_room1_out
0
= 93
1
99 0
0
0
1
0 0 0 2 0 1
1
0 0 0 5 -1 0
end_operator
begin_operator
drive-base doorway0_room1_out doorway0_room1
0
= 93
1
96 0
0
0
1
0 0 0 3 0 1
1
0 0 0 2 -1 0
end_operator
begin_operator
drive-base doorway0_room1_out doorway1_room1
0
= 93
1
98 0
0
0
1
0 0 0 3 0 1
1
0 0 0 4 -1 0
end_operator
begin_operator
drive-base doorway0_room1_out doorway1_room1_out
0
= 93
1
99 0
0
0
1
0 0 0 3 0 1
1
0 0 0 5 -1 0
end_operator
begin_operator
drive-base doorway1_room1 doorway0_room1
0
= 93
1
96 0
0
0
1
0 0 0 4 0 1
1
0 0 0 2 -1 0
end_operator
begin_operator
drive-base doorway1_room1 doorway0_room1_out
0
= 93
1
97 0
0
0
1
0 0 0 4 0 1
1
0 0 0 3 -1 0
end_operator
begin_operator
drive-base doorway1_room1 doorway1_room1_out
0
= 93
1
99 0
0
0
1
0 0 0 4 0 1
1
0 0 0 5 -1 0
end_operator
begin_operator
drive-base doorway1_room1_out doorway0_room1
0
= 93
1
96 0
0
0
1
0 0 0 5 0 1
1
0 0 0 2 -1 0
end_operator
begin_operator
drive-base doorway1_room1_out doorway0_room1_out
0
= 93
1
97 0
0
0
1
0 0 0 5 0 1
1
0 0 0 3 -1 0
end_operator
begin_operator
drive-base doorway1_room1_out doorway1_room1
0
= 93
1
98 0
0
0
1
0 0 0 5 0 1
1
0 0 0 4 -1 0
end_operator
begin_operator
drive-base doorway1_room2 doorway1_room2_out
0
= 93
1
101 0
0
0
1
0 0 0 6 0 1
1
0 0 0 7 -1 0
end_operator
begin_operator
drive-base doorway1_room2 object_location_loc1_room2
0
= 93
1
104 0
0
0
1
0 0 0 6 0 1
1
0 0 0 10 -1 0
end_operator
begin_operator
drive-base doorway1_room2 search_location_loc0_room2
0
= 93
1
106 0
0
0
1
0 0 0 6 0 1
1
0 0 0 12 -1 0
end_operator
begin_operator
drive-base doorway1_room2_out doorway1_room2
0
= 93
1
100 0
0
0
1
0 0 0 7 0 1
1
0 0 0 6 -1 0
end_operator
begin_operator
drive-base doorway1_room2_out object_location_loc1_room2
0
= 93
1
104 0
0
0
1
0 0 0 7 0 1
1
0 0 0 10 -1 0
end_operator
begin_operator
drive-base doorway1_room2_out search_location_loc0_room2
0
= 93
1
106 0
0
0
1
0 0 0 7 0 1
1
0 0 0 12 -1 0
end_operator
begin_operator
drive-base init_loc doorway0_room0
0
= 93
1
94 0
0
0
1
0 0 0 8 0 1
1
0 0 0 0 -1 0
end_operator
begin_operator
drive-base init_loc doorway0_room0_out
0
= 93
1
95 0
0
0
1
0 0 0 8 0 1
1
0 0 0 1 -1 0
end_operator
begin_operator
drive-base init_loc object_location_loc0_room0
0
= 93
1
103 0
0
0
1
0 0 0 8 0 1
1
0 0 0 9 -1 0
end_operator
begin_operator
drive-base init_loc robot_location
0
= 93
2
105 0
54 0
0
0
1
0 0 0 8 0 1
1
0 0 0 11 -1 0
end_operator
begin_operator
drive-base init_loc search_location_loc1_room0
0
= 93
1
107 0
0
0
1
0 0 0 8 0 1
1
0 0 0 13 -1 0
end_operator
begin_operator
drive-base object_location_loc0_room0 doorway0_room0
0
= 93
1
94 0
0
0
1
0 0 0 9 0 1
1
0 0 0 0 -1 0
end_operator
begin_operator
drive-base object_location_loc0_room0 doorway0_room0_out
0
= 93
1
95 0
0
0
1
0 0 0 9 0 1
1
0 0 0 1 -1 0
end_operator
begin_operator
drive-base object_location_loc0_room0 init_loc
0
= 93
1
102 0
0
0
1
0 0 0 9 0 1
1
0 0 0 8 -1 0
end_operator
begin_operator
drive-base object_location_loc0_room0 robot_location
0
= 93
2
60 0
105 0
0
0
1
0 0 0 9 0 1
1
0 0 0 11 -1 0
end_operator
begin_operator
drive-base object_location_loc0_room0 search_location_loc1_room0
0
= 93
1
107 0
0
0
1
0 0 0 9 0 1
1
0 0 0 13 -1 0
end_operator
begin_operator
drive-base object_location_loc1_room2 doorway1_room2
0
= 93
1
100 0
0
0
1
0 0 0 10 0 1
1
0 0 0 6 -1 0
end_operator
begin_operator
drive-base object_location_loc1_room2 doorway1_room2_out
0
= 93
1
101 0
0
0
1
0 0 0 10 0 1
1
0 0 0 7 -1 0
end_operator
begin_operator
drive-base object_location_loc1_room2 search_location_loc0_room2
0
= 93
1
106 0
0
0
1
0 0 0 10 0 1
1
0 0 0 12 -1 0
end_operator
begin_operator
drive-base robot_location doorway0_room0
0
= 93
1
94 0
0
0
1
0 0 0 11 0 1
1
0 0 0 0 -1 0
end_operator
begin_operator
drive-base robot_location doorway0_room0_out
0
= 93
1
95 0
0
0
1
0 0 0 11 0 1
1
0 0 0 1 -1 0
end_operator
begin_operator
drive-base robot_location init_loc
0
= 93
1
102 0
0
0
1
0 0 0 11 0 1
1
0 0 0 8 -1 0
end_operator
begin_operator
drive-base robot_location object_location_loc0_room0
0
= 93
1
103 0
0
0
1
0 0 0 11 0 1
1
0 0 0 9 -1 0
end_operator
begin_operator
drive-base robot_location search_location_loc1_room0
0
= 93
1
107 0
0
0
1
0 0 0 11 0 1
1
0 0 0 13 -1 0
end_operator
begin_operator
drive-base search_location_loc0_room2 doorway1_room2
0
= 93
1
100 0
0
0
1
0 0 0 12 0 1
1
0 0 0 6 -1 0
end_operator
begin_operator
drive-base search_location_loc0_room2 doorway1_room2_out
0
= 93
1
101 0
0
0
1
0 0 0 12 0 1
1
0 0 0 7 -1 0
end_operator
begin_operator
drive-base search_location_loc0_room2 object_location_loc1_room2
0
= 93
1
104 0
0
0
1
0 0 0 12 0 1
1
0 0 0 10 -1 0
end_operator
begin_operator
drive-base search_location_loc1_room0 doorway0_room0
0
= 93
1
94 0
0
0
1
0 0 0 13 0 1
1
0 0 0 0 -1 0
end_operator
begin_operator
drive-base search_location_loc1_room0 doorway0_room0_out
0
= 93
1
95 0
0
0
1
0 0 0 13 0 1
1
0 0 0 1 -1 0
end_operator
begin_operator
drive-base search_location_loc1_room0 init_loc
0
= 93
1
102 0
0
0
1
0 0 0 13 0 1
1
0 0 0 8 -1 0
end_operator
begin_operator
drive-base search_location_loc1_room0 object_location_loc0_room0
0
= 93
1
103 0
0
0
1
0 0 0 13 0 1
1
0 0 0 9 -1 0
end_operator
begin_operator
drive-base search_location_loc1_room0 robot_location
0
= 93
2
80 0
105 0
0
0
1
0 0 0 13 0 1
1
0 0 0 11 -1 0
end_operator
begin_operator
drive-through-doorway doorway0 doorway0_room0 doorway0_room1_out
0
= 92
1
82 0
0
0
1
0 0 0 0 0 1
2
0 0 1 82 0 82 -1 1
0 0 0 3 -1 0
end_operator
begin_operator
drive-through-doorway doorway0 doorway0_room1 doorway0_room0_out
0
= 92
1
82 0
0
0
1
0 0 0 2 0 1
2
0 0 0 1 -1 0
0 0 1 82 0 82 -1 1
end_operator
begin_operator
drive-through-doorway doorway1 doorway1_room1 doorway1_room2_out
0
= 92
1
83 0
0
0
1
0 0 0 4 0 1
2
0 0 1 83 0 83 -1 1
0 0 0 7 -1 0
end_operator
begin_operator
drive-through-doorway doorway1 doorway1_room2 doorway1_room1_out
0
= 92
1
83 0
0
0
1
0 0 0 6 0 1
2
0 0 1 83 0 83 -1 1
0 0 0 5 -1 0
end_operator
2
begin_rule
2
85 1
86 1
84 1 0
end_rule
begin_rule
2
89 0
90 0
86 0 1
end_rule
0
0
