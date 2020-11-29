source ~/.bashrc
export SCRIPT_PATH=`pwd`
#source ~/ambf/build/devel/setup.bash
cd ~/WPI/thesis/git/ambf/bin/lin-x86_64
./ambf_simulator -a ${SCRIPT_PATH}/exohuman_v3.yaml
