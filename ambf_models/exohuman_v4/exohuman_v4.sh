source ~/.bashrc
echo "current path"
export SCRIPT_DIR=`pwd`
#source ~/ambf/build/devel/setup.bash
cd ~/WPI/thesis/git/ambf/bin/lin-x86_64
./ambf_simulator -a ${SCRIPT_DIR}/exohuman_v4.yaml -t 1 -p 60
