ROOT_DIR=$(CURDIR)/../..
include $(ROOT_DIR)/make.conf
INCLUDE_DIR=$(ROOT_DIR)/src/

RML_DIR=/opt/ros/groovy/

ifeq (,$(wildcard $(RML_DIR)))
default: no_rml
else
default: rml_pos.so
endif

.PHONY: no_rml

REFLEXXES_LIB=${RML_DIR}/lib
REFLEXXES_INCLUDE=${RML_DIR}/include

TYPES:=$(wildcard types/*.h)
HEXARRS:=$(TYPES:%=%.hexarr)

rml_pos.so: rml_pos.o $(INCLUDE_DIR)/libubx.so
	${CPP} $(CFLAGS_SHARED) -o rml_pos.so rml_pos.o ${INCLUDE_DIR}/libubx.so -L${REFLEXXES_LIB} -lreflexxes_type2

rml_pos.o: rml_pos.hpp rml_pos.cpp $(INCLUDE_DIR)/ubx.h $(INCLUDE_DIR)/ubx_types.h $(INCLUDE_DIR)/ubx.c $(HEXARRS)
	${CPP} -fPIC -I$(INCLUDE_DIR) -I${REFLEXXES_INCLUDE} -c $(CFLAGS) rml_pos.cpp

no_rml:
	$(warning No 'reflexxes' symlink found, not building RML trajectory generator)
	@:

clean:
	rm -f *.o *.so *~ core $(HEXARRS)
