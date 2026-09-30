cmd () {
    rostopic pub -1 /so101/$1/cmd_pos std_msgs/Float64 "data: $2"
}

echo "===== shoulder_pan ====="
cmd shoulder_pan 0.3
sleep 2
cmd shoulder_pan -0.3
sleep 2
cmd shoulder_pan -0.07
sleep 2

echo "===== shoulder_lift ====="
cmd shoulder_lift -1.0
sleep 2
cmd shoulder_lift -0.7
sleep 2
cmd shoulder_lift -1.175
sleep 2

echo "===== elbow_flex ====="
cmd elbow_flex 1.4
sleep 2
cmd elbow_flex 1.0
sleep 2
cmd elbow_flex 1.4
sleep 2

echo "===== wrist_flex ====="
cmd wrist_flex 1.1
sleep 2
cmd wrist_flex 0.7
sleep 2
cmd wrist_flex 1.361
sleep 2

echo "===== wrist_roll ====="
cmd wrist_roll -0.8
sleep 2
cmd wrist_roll -0.3
sleep 2
cmd wrist_roll -1.134
sleep 2

echo "===== gripper ====="
cmd gripper 0.3
sleep 2
cmd gripper 0.6
sleep 2
cmd gripper 1.0
sleep 2
cmd gripper 0.3

echo "===== TEST FINISHED ====="