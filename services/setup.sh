
# move the directories
mv /home/ubuntu/ros2_ws/src/ros2_for_waveshare_alphabot2/services/lirc /home/ubuntu/lirc
mv /home/ubuntu/ros2_ws/src/ros2_for_waveshare_alphabot2/services/wsRGB /home/ubuntu/wsRGB
mv /home/ubuntu/ros2_ws/src/ros2_for_waveshare_alphabot2/services/setup /home/ubuntu/setup

# make the needed files executable
chmod +x /home/ubuntu/setup/reinstall_alphabot.sh
chmod +x /home/ubuntu/setup/full_reinstall_alphabot.sh 
chmod +x /home/ubuntu/setup/reinstall_camera.sh
chmod +x /home/ubuntu/setup/full_reinstall_camera.sh 
chmod +x /home/ubuntu/setup/create_workspace.sh 
chmod +x /home/ubuntu/setup/install_ros2_humble.sh 

# file to set up RGB LED service 
chmod +x /home/ubuntu/wsRGB/setup.sh

while true; do
  echo 'Do you want to run wsRGB/setup.sh to set up the RGB LED service? [y/n]'
  read INPUT_STRING
  case $INPUT_STRING in
    [yY])
        cd /home/ubuntu/wsRGB
        sudo ./setup.sh
        break
        ;;
    [nN])
        break
        ;;
    *)
        echo "Wrong input. Please enter 'y' or 'n'."
        ;;
  esac
done

echo "setup sucsessfull"