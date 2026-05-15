# Term Project
## Main goal
* Drive along the given path
    * Given path is open at testing day

### Rule
* Low time -> High score
* 2 seconds added if you hit obstacles

### Path composition
* Partition walls with gaps
* Few obstacles
* A room with an enclosed structure

## Given materials
### TurtleBot3 Burger
* max linear speed: 0.22 (m/s)
    * only linear.x
* max rotational speed: 2.84 (rad/s)
    * only angular.z
    
### Available sensors & Interfaces
* OpenCR board (IMU, Gyro, Magnetometer)
* Jetson Nano
* Realsense D435i
* SLAMTEC RPLIDAR C1
* Turtlebot default LIDAR