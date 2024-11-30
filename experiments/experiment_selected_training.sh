for i in `seq 10`
#for i in `seq 2`
do
  #roslaunch nav_cloning nav_cloning_sim.launch mode:=selected_training
  roslaunch nav_cloning nav_cloning_square_road.launch
  sleep 10
done
