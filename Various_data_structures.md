이 문서는 [VINS-Mono代码解读——各种数据结构 sensor_msgs](https://blog.csdn.net/qq_41839222/article/details/86030962)를 영어로 번역하여 정리하였습니다.  


## Standard msg structure

### 1. sensor_msgs::ImageConstPtr Definition
---
std_msgs/Header header  
uint32 height            
uint32 width            
string encoding  
uint8 is_bigendian  
uint32 step  
uint8[] data  

---

`feature_trackers_node.cpp`파일 안에 `img_callback()`에서 사용

Ref : http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html



### 2. sensor_msgs::PointCloudPtr Definition
---
std_msgs/Header header  
geometry_msgs/Point32[] points  
sensor_msgs/ChannelFloat32[] channels   

---

`feature_trackers_node.cpp`파일 안에 `img_callback()`에서 사용  

Ref : http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud.html

### 3. sensor_msgs::PointCloud msg_match_points Definition

sensor_msgs::PointCloudPtr과 비슷한 구조 그러나 channels가 다르다.  

`bool KeyFrame::findConnection()`에서 msg_match_points를 만든다.  
그리고 `pose_graph_node.cpp`파일에서 topic "/pose_graph/match_points" Publish 해준다.  

"/pose_graph/match_points" topic은 `estimator_node.cpp` 파일의 `main()`에서 subscribe 된다.  

### 4. sensor_msgs::ImuConstPtr
---
Header header	

geometry_msgs/Quaternion orientation	# [x,y,z,w]
float64[9] orientation_covariance		# Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity	#   [x,y,z] 
float64[9] angular_velocity_covariance	#  Row major(   ) about x, y, z axes

geometry_msgs/Vector3 linear_acceleration	#  [x,y,z]
float64[9] linear_acceleration_covariance

---

`estimator_node.cpp`파일에 있는 `getMeasurements()`에서 사용  
Timestamp를 이용하여 Image와 IMU data들을 align시킨다.

## VINS-MONO code에서 사용하고 있는 structure

### 1. `Measurements`  

```cpp
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
```

`estimator_node.cpp`파일에 있는 `getMeasurements()`에서 사용   

sensor_msgs::PointCloudConstPtr>은 feature_points를 의미하고,  
std::vector< sensor_msgs::ImuConstPtr>는 이전 frame과 현재 frame 사이에 IMU data를 의미한다.  

두 구조를 결합하고, 저장을 하기 위해 vector 자료구조를 사용합니다.  

### 2. `map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image`  

`estimator_node.cpp` 파일에 있는 `prcess()`에서 생성하고, `estimator.processImage(image, img_msg->header)`
`estimator.cpp` 파일에 있는 Estimator::processImage()에서 호출  

feature_id가 인덱스로 있고, 각각의 feature point  (camera_id, [x, y, z, u, v, vx, vy])로 구성되어 있는 map 자료구조.  

### 3. `map<double, ImageFrame> all_image_frame`  

`estimator.h` 파일에 존재  
double에 해당되는건 image frame의 Timestamp  
ImageFrame은 여기서 만든 class에 해당한다.  
