# jszr_forward

## jszr_tf

接收定位或者mc发送的odom，发送tf(map——>odom;  odom——>base_link)

### 采用定位tf:

`ros2 launch pub_tf pub_tf.launch.py tf_type:=localization_tf`

### 采用MC tf:

ros2 launch pub_tf pub_tf.launch.py tf_type:=mc_tf
