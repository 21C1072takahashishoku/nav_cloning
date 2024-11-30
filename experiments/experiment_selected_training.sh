<<<<<<< HEAD
for i in `seq 10`
#for i in `seq 2`
=======
for i in `seq 1`
>>>>>>> 7708ec1d40aa1fa7033accb2c6659e4537e1d56a
do
  #roslaunch nav_cloning nav_cloning_sim.launch mode:=selected_training
  roslaunch nav_cloning nav_cloning_square_road.launch
  sleep 10
done
