# brute force clean and compile file and define ld
  export LD_LIBRARY_PATH="/home/ubuntu/kobuki/install/lib":"/home/ubuntu/realsense/librealsense/build/release/":"./"
  g++ -c tustin.cpp -I./
  g++ -c control.cpp -fpermissive -I./  
  g++ -c kobuki_with_rs_program.cpp -fpermissive -I/home/ubuntu/kobuki/install/include -I/home/ubuntu/kobuki/install/include/eigen3 -I/home/ubuntu/realsense/librealsense/include/ -I/home/ubuntu/realsense/librealsense/examples/C/ -I/home/ubuntu/realsense/librealsense/include/ -I/home/ubuntu/realsense/librealsense/third-party/ -I./
  g++ -o kobuki_with_rs_program kobuki_with_rs_program.o control.o tustin.o -lrt -lm -lpthread -lkobuki_core -lecl_geometry -lecl_threads -lecl_time -lecl_exceptions -lecl_time_lite -lecl_type_traits -lrealsense2  -L/home/ubuntu/kobuki/install/lib/ 
 

