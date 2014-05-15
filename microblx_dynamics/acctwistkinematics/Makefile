ROOT_DIR=$(CURDIR)/../..
include $(ROOT_DIR)/make.conf
CFLAGS:=$(CFLAGS) -Wno-unused-function


UBX_DIR=$(ROOT_DIR)/src/
EIGEN_DIR=/usr/include/eigen3/
CUSTOM_TYPES=$(ROOT_DIR)/microblx_dynamics/acctwistkinematics/types/
KDL_TYPES=$(ROOT_DIR)/std_types/kdl/types/
YOUBOT_DRIVER_DIR=$(ROOT_DIR)/microblx_dynamics/youbot_driver/types/
KDL_PARSER=/opt/ros/groovy/include
KDL_PARSER_LIB=/opt/ros/groovy/lib
HAVE_KDL=/opt/ros/groovy/stacks/orocos_kinematics_dynamics/orocos_kdl/install_dir

INCLUDES= -I$(UBX_DIR) -I$(EIGEN_DIR) -I$(YOUBOT_DRIVER_DIR) -I$(KDL_TYPES) -I$(KDL_PARSER)

TYPES:=$(wildcard types/*.h)
HEXARRS:=$(TYPES:%=%.hexarr)

forward_acctwist_kin.so: forward_acctwist_kin.o $(UBX_DIR)/libubx.so
	${CPP} $(CFLAGS_SHARED) -o forward_acctwist_kin.so forward_acctwist_kin.o $(UBX_DIR)/libubx.so -L${HAVE_KDL}/lib -lorocos-kdl -L${KDL_PARSER_LIB} -lkdl_parser

forward_acctwist_kin.o: forward_acctwist_kin.cpp forward_acctwist_kin.hpp $(UBX_DIR)/ubx.h $(UBX_DIR)/ubx_types.h $(UBX_DIR)/ubx.c $(HEXARRS)
	${CPP} -fPIC $(INCLUDES) -c $(CFLAGS) forward_acctwist_kin.cpp

clean:
	rm -f *.o *.so *~ core $(HEXARRS)
