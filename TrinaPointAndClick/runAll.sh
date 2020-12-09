cd ~/iml-internal/Ebolabot
python Common/system_state_service.py &
./MotionServer_physical &
./ControllerDispatcher &
source ~/UBIROS_prototype_code/build/devel/setup.bash
roslaunch gentle_ros gentle_ros_launcher1.launch
cd ~/TrinaPointAndClick

