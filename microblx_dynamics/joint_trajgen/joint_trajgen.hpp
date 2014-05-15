/*
 * joint_trajgen microblx function block (autogenerated, don't edit)
 */

#define NR_OF_JOINTS 5
#define POLYNOMIAL_DEGREE 4

#include "ubx.h"
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarrayvel.hpp>

/* from youbot_driver */
#include <motionctrl_jnt_state.h>

/* from std_types/kdl */
#include <kdl.h>

/* includes types and type metadata */

ubx_type_t types[] = {
	{ NULL },
};

/* block meta information */
char joint_trajgen_meta[] =
	" { doc='',"
	"   license='',"
	"   real-time=true,"
	"}";

/* declaration of block configuration */
ubx_config_t joint_trajgen_config[] = {
	{ .name="time_duration", .type_name = "double", .doc="duration of the trajectory" },
	{ .name="des_target_pos", .type_name = "double", .value = {.len=NR_OF_JOINTS}, .doc="desired traget joint positions"},
	{ NULL },
};

/* declaration port block ports */
ubx_port_t joint_trajgen_ports[] = {
	{ .name="msr_pos", .in_type_name="double", .in_data_len=NR_OF_JOINTS, .doc="current measured position"  },
	{ .name="msr_vel", .in_type_name="double", .in_data_len=5, .doc="current measured velocity"  },
	{ .name="des_pos", .out_type_name="double", .out_data_len=5, .doc="current desired position"  },
	{ .name="des_vel", .out_type_name="double", .out_data_len=5, .doc="current desired velocity"  },
	{ .name="reached", .out_type_name="int", .out_data_len=1, .doc="the final state has been reached"  },
	{ NULL },
};

/* declare a struct port_cache */
struct joint_trajgen_port_cache {
	ubx_port_t* msr_pos;
	ubx_port_t* msr_vel;
	ubx_port_t* des_pos;
	ubx_port_t* des_vel;
	ubx_port_t* reached;
};

/* declare a helper function to update the port cache this is necessary
 * because the port ptrs can change if ports are dynamically added or
 * removed. This function should hence be called after all
 * initialization is done, i.e. typically in 'start'
 */
static void update_port_cache(ubx_block_t *b, struct joint_trajgen_port_cache *pc)
{
	pc->msr_pos = ubx_port_get(b, "msr_pos");
	pc->msr_vel = ubx_port_get(b, "msr_vel");
	pc->des_pos = ubx_port_get(b, "des_pos");
	pc->des_vel = ubx_port_get(b, "des_vel");
	pc->reached = ubx_port_get(b, "reached");
}


/* for each port type, declare convenience functions to read/write from ports */
def_read_arr_fun(read_msr_pos, double, NR_OF_JOINTS)
def_read_arr_fun(read_msr_vel, double, NR_OF_JOINTS)
def_write_arr_fun(write_des_pos, double, NR_OF_JOINTS)
def_write_arr_fun(write_des_vel, double, NR_OF_JOINTS)
def_write_fun(write_reached, int)

/* block operation forward declarations */
int joint_trajgen_init(ubx_block_t *b);
int joint_trajgen_start(ubx_block_t *b);
void joint_trajgen_stop(ubx_block_t *b);
void joint_trajgen_cleanup(ubx_block_t *b);
void joint_trajgen_step(ubx_block_t *b);


/* put everything together */
ubx_block_t joint_trajgen_block = {
	.name = "joint_trajgen",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = joint_trajgen_meta,
	.configs = joint_trajgen_config,
	.ports = joint_trajgen_ports,

	/* ops */
	.init = joint_trajgen_init,
	.start = joint_trajgen_start,
	.stop = joint_trajgen_stop,
	.cleanup = joint_trajgen_cleanup,
	.step = joint_trajgen_step,
};


/* joint_trajgen module init and cleanup functions */
int joint_trajgen_mod_init(ubx_node_info_t* ni)
{
	DBG(" ");
	int ret = -1;
	ubx_type_t *tptr;

	for(tptr=types; tptr->name!=NULL; tptr++) {
		if(ubx_type_register(ni, tptr) != 0) {
			goto out;
		}
	}

	if(ubx_block_register(ni, &joint_trajgen_block) != 0)
		goto out;

	ret=0;
out:
	return ret;
}

void joint_trajgen_mod_cleanup(ubx_node_info_t *ni)
{
	DBG(" ");
	const ubx_type_t *tptr;

	for(tptr=types; tptr->name!=NULL; tptr++)
		ubx_type_unregister(ni, tptr->name);

	ubx_block_unregister(ni, "joint_trajgen");
}

/* declare module init and cleanup functions, so that the ubx core can
 * find these when the module is loaded/unloaded */
UBX_MODULE_INIT(joint_trajgen_mod_init)
UBX_MODULE_CLEANUP(joint_trajgen_mod_cleanup)
