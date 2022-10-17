# AirSimTest

code collection to interact with AirSim
note : to get parameter of AirSim camera use this code : 

```
client.simGetCameraInfo(str(camera_id))
```

# ardupilot 
output add 192.168.1.18:14560
param load ../Tools/autotest/default_params/airsim-quadX.parm

# references
Computer vision book http://szeliski.org/Book/

Aruco introduction
https://machinelearningknowledge.ai/augmented-reality-using-aruco-marker-detection-with-python-opencv/

https://stackoverflow.com/questions/57839713/how-can-i-calculate-the-yaw-of-aruco-marker

https://stackoverflow.com/questions/51584562/calculating-the-distance-and-yaw-between-aruco-marker-and-camera

https://docs.opencv.org/3.4/d5/d1f/calib3d_solvePnP.html
https://www.delftstack.com/fr/howto/python/opencv-solvepnp/#:~:text=La%20fonction%20solvepnp%20%28%29%20de%20la%20biblioth%C3%A8que%20OpenCV,de%20l%E2%80%99objet%20avec%20la%20matrice%20de%20la%20cam%C3%A9ra.

control
https://github.com/python-control/python-control

