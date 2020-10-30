source ~/.bashrc
source ~/ambf/build/devel/setup.bash
cd ~/ambf/bin/lin-x86_64
./ambf_simulator -a  ~/catkin_ws/src/ambf_walker/ambf_models/simulator/exo.yaml,/home/nathanielgoldfarb/catkin_ws/src/ambf_walker/ambf_models/stair/stairs.yaml
#./ambf_simulator -a ~/catkin_ws/src/ambf_walker/ambf_models/LARRE2.yaml
