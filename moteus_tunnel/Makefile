CANARD_BASE=libcanard

CXX=g++
CC=gcc
CFLAGS = -g -Wall -Idsdl_generated/include -I$(CANARD_BASE) -I$(CANARD_BASE)/drivers/socketcan

# enable C++ API
CFLAGS += -DDRONECAN_CXX_WRAPPERS
CFLAGS += -DNUM_CANARD_HANDLERS=16

LIBS_C=$(CANARD_BASE)/canard.c $(CANARD_BASE)/drivers/socketcan/socketcan.c
LIBS_C_OBJ := $(LIBS_C:.c=.o)

# add in generated code
LIBS+=dsdl_generated/src/uavcan.protocol.NodeStatus.c
LIBS+=dsdl_generated/src/uavcan.protocol.GetNodeInfo_res.c
LIBS+=dsdl_generated/src/uavcan.protocol.GetNodeInfo_req.c
LIBS+=dsdl_generated/src/uavcan.equipment.esc.RawCommand.c
LIBS+=dsdl_generated/src/uavcan.equipment.esc.Status.c
LIBS+=dsdl_generated/src/uavcan.protocol.dynamic_node_id.Allocation.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.GetSet_req.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.GetSet_res.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.ExecuteOpcode_req.c
LIBS+=dsdl_generated/src/uavcan.protocol.param.ExecuteOpcode_res.c
LIBS+=dsdl_generated/src/uavcan.tunnel.Broadcast.c

LIBS_OBJ := $(LIBS:.c=.o)

all: moteus_tunnel

libcanard:
	git clone --depth 1 https://github.com/dronecan/libcanard

DSDL:
	git clone --depth 1 https://github.com/DroneCAN/DSDL

dronecan_dsdlc:
	git clone --depth 1 https://github.com/DroneCAN/dronecan_dsdlc

dsdl_generated: DSDL dronecan_dsdlc
	python3 dronecan_dsdlc/dronecan_dsdlc.py -O dsdl_generated DSDL/dronecan DSDL/uavcan DSDL/com DSDL/ardupilot

moteus_tunnel: libcanard dsdl_generated moteus_tunnel.cpp $(LIBS_OBJ) $(LIBS_C_OBJ)
	$(CXX) -o moteus_tunnel moteus_tunnel.cpp $(CFLAGS) $(LIBS_OBJ) $(LIBS_C_OBJ)

clean:
	rm -rf esc_node DSDL dsdl_generated dronecan_dsdlc