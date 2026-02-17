close all 
clear
clc 


cam_data = read_camera_data("data/camera.dat");


traj_data = read_traj_data("data/trajectory.dat");


draw_traj(traj_data);
disp("Map displayed. Press any key to close and exit");
pause;