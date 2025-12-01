#for i in `seq 10`
for i in `seq 10`
do
  roslaunch nav_cloning nav_cloning_4chdrive.launch
  #roslaunch nav_cloning nav_cloning_sim.launch 
  #roslaunch nav_cloning nav_cloning_Tsudanuma_2-3.launch mode:=use_dl_output
  sleep 10
done
