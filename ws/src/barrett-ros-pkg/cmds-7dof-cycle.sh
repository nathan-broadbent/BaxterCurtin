#!/bin/bash
# sh cmds-7dof-cycle.sh <number of cycles>

echo "Cycling $1 times."

for i in `seq 1 $1`;
do
  echo "Iteration: $i"
  
  echo "Closing BHand grasp"
  rosservice call /bhand/finger_pos '[2, 2, 2]'

  echo "Sending joint_move command"
  rosservice call /wam/joint_move '[0, -2, 0, 2.5, 0, 0, 0]'
  
  echo "Waiting for move_is_done..."
  until rostopic echo -n 1 /wam/move_is_done |grep True; do
    sleep 1
  done

  echo "Opening BHand grasp"
  rosservice call /bhand/finger_pos '[0, 0, 0]'
  
  echo "Sending joint_move command"
  rosservice call /wam/joint_move '[0.1, -1.8, -0.1, 1.57, -0.1, 0.1, 0.1]'

  echo "Waiting for move_is_done..."
  until rostopic echo -n 1 /wam/move_is_done |grep True; do
    sleep 1
  done
done

echo "Returning to home position"
# Note: move_is_done is unaffected by go_home
rosservice call /wam/go_home 

echo "Done."
