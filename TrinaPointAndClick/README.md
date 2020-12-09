# Trina Point and Click

run four ssh terminals to trina:
    cd ~/iml-internal/Ebolabot
        python Common/system_state_service.py
        ./MotionServer_physical
        ./ControllerDispatcher 
    source ~/UBIROS_prototype_code/build/devel/setup.bash
        roslaunch gentle_ros gentle_ros_launcher1.launch
          (if this doesnt work, run this and try again)
           sudo udevadm control --reload && sudo udevadm trigger

run one ssh -X terminal to trina:
    source ~/TrinaPointAndClick/devel/setup.bash
    roslaunch TrinaPointAndClick TrinaPointAndClickUI.launch 

on desktop:
    cd ~/iml-internal/Ebolabot 
    source ~/TrinaPointAndClick/build/devel/setup.bash
        ./TaskGUIDemo

        

