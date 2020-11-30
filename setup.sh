sudo apt install libeigen3-dev
sudo apt install ros-melodic-smach
pip3 install git+https://github.com/WPI-AIM/AIM_GaitAnalysisToolkit.gi

# shellcheck disable=SC2164
cd lib
git clone https://github.com/WPI-AIM-Hybrid-Exoskeleton/ilqr.git
cd ilqr && python setup.py install
cd ..
# shellcheck disable=SC2164
git clone https://github.com/WPI-AIM-Hybrid-Exoskeleton/rbdl-orb.git
# shellcheck disable=SC2164
cd rbdl-orb
mkdir rbdl-build
cd rbdl-build/ || exit
cmake -D RBDL_BUILD_PYTHON_WRAPPER=True CMAKE_BUILD_TYPE=Release ../
make
sudo make install
cd ./python
sudo cp rbdl.so /usr/local/lib/python3.8/dist-packages