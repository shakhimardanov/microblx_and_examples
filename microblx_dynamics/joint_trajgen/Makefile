ROOT_DIR=$(CURDIR)/../..
include $(ROOT_DIR)/make.conf
CFLAGS:=$(CFLAGS) -Wno-unused-function


UBX_DIR=$(ROOT_DIR)/src/
EIGEN_DIR=/usr/include/eigen3/
CUSTOM_TYPES=$(ROOT_DIR)/microblx_dynamics/posekinematics/types/
KDL_TYPES=$(ROOT_DIR)/std_types/kdl/types/
YOUBOT_DRIVER_DIR=$(ROOT_DIR)/microblx_dynamics/youbot_driver/types/
HAVE_KDL=/opt/ros/groovy/stacks/orocos_kinematics_dynamics/orocos_kdl/install_dir

INCLUDES= -I$(UBX_DIR) -I$(EIGEN_DIR) -I$(YOUBOT_DRIVER_DIR) -I$(KDL_TYPES) 
TYPES:=$(wildcard types/*.h)
HEXARRS:=$(TYPES:%=%.hexarr)

joint_trajgen.so: joint_trajgen.o $(UBX_DIR)/libubx.so
	${CPP} $(CFLAGS_SHARED) -o joint_trajgen.so joint_trajgen.o ${UBX_DIR}/libubx.so -L${HAVE_KDL}/lib -lorocos-kdl 

joint_trajgen.o: joint_trajgen.cpp joint_trajgen.hpp $(UBX_DIR)/ubx.h $(UBX_DIR)/ubx_types.h $(UBX_DIR)/ubx.c $(HEXARRS)
	${CPP} -fPIC $(INCLUDES) -c $(CFLAGS) joint_trajgen.cpp

clean:
	rm -f *.o *.so *~ core $(HEXARRS)
