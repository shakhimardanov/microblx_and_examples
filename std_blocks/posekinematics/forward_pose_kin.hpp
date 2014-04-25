/*
 * forward_pose_kin microblx function block (autogenerated, don't edit)
 */

#include <ubx.h>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>

/* from youbot_driver */
#include <motionctrl_jnt_state.h>

/* from std_types/kdl */
#include <kdl.h>

/* includes types and type metadata */
#include "types/vector.h"
#include "types/vector.h.hexarr"
#include "types/robot_data.h"
#include "types/robot_data.h.hexarr"


#define NR_OF_JOINTS 5
ubx_type_t types[] = {
	def_struct_type(struct vector, &vector_h),
	def_struct_type(struct robot_data, &robot_data_h),
	{ NULL },
};

/* block meta information */
char forward_pose_kin_meta[] =
	" { doc='',"
	"   real-time=true,"
	"}";

/* declaration of block configuration */
ubx_config_t forward_pose_kin_config[] = {
	{ .name="robot_structural_model", .type_name = "char", .doc="file containing urdf model of a robot " },
	{ NULL },
};

/* declaration port block ports */
ubx_port_t forward_pose_kin_ports[] = {
	{ .name="input_joint_poses", .in_type_name="double", .in_data_len=NR_OF_JOINTS, .doc=""  },
	{ .name="output_link_poses", .out_type_name="struct kdl_frame", .out_data_len=NR_OF_JOINTS, .doc=""  },
	{ .name="output_ee_pose", .out_type_name="struct kdl_frame", .out_data_len=1, .doc=""  },
	{ NULL },
};

/* declare a struct port_cache */
struct forward_pose_kin_port_cache {
	ubx_port_t* input_joint_poses;
	ubx_port_t* output_link_poses;
	ubx_port_t* output_ee_pose;
};

/* declare a helper function to update the port cache this is necessary
 * because the port ptrs can change if ports are dynamically added or
 * removed. This function should hence be called after all
 * initialization is done, i.e. typically in 'start'
 */
static void update_port_cache(ubx_block_t *b, struct forward_pose_kin_port_cache *pc)
{
	pc->input_joint_poses = ubx_port_get(b, "input_joint_poses");
	pc->output_link_poses = ubx_port_get(b, "output_link_poses");
	pc->output_ee_pose = ubx_port_get(b, "output_ee_pose");
}


/* for each port type, declare convenience functions to read/write from ports */
def_read_arr_fun(read_joint_poses, double, NR_OF_JOINTS)
def_write_arr_fun(write_link_poses, struct kdl_frame, NR_OF_JOINTS)
def_write_fun(write_ee_pose, struct kdl_frame)

/* block operation forward declarations */
int forward_pose_kin_init(ubx_block_t *b);
int forward_pose_kin_start(ubx_block_t *b);
void forward_pose_kin_stop(ubx_block_t *b);
void forward_pose_kin_cleanup(ubx_block_t *b);
void forward_pose_kin_step(ubx_block_t *b);


/* put everything together */
ubx_block_t forward_pose_kin_block = {
	.name = "forward_pose_kin",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = forward_pose_kin_meta,
	.configs = forward_pose_kin_config,
	.ports = forward_pose_kin_ports,

	/* ops */
	.init = forward_pose_kin_init,
	.start = forward_pose_kin_start,
	.stop = forward_pose_kin_stop,
	.cleanup = forward_pose_kin_cleanup,
	.step = forward_pose_kin_step,
};


/* forward_pose_kin module init and cleanup functions */
int forward_pose_kin_mod_init(ubx_node_info_t* ni)
{
	DBG(" ");
	int ret = -1;
	ubx_type_t *tptr;

	for(tptr=types; tptr->name!=NULL; tptr++) {
		if(ubx_type_register(ni, tptr) != 0) {
			goto out;
		}
	}

	if(ubx_block_register(ni, &forward_pose_kin_block) != 0)
		goto out;

	ret=0;
out:
	return ret;
}

void forward_pose_kin_mod_cleanup(ubx_node_info_t *ni)
{
	DBG(" ");
	const ubx_type_t *tptr;

	for(tptr=types; tptr->name!=NULL; tptr++)
		ubx_type_unregister(ni, tptr->name);

	ubx_block_unregister(ni, "forward_pose_kin");
}

/* declare module init and cleanup functions, so that the ubx core can
 * find these when the module is loaded/unloaded */
UBX_MODULE_INIT(forward_pose_kin_mod_init)
UBX_MODULE_CLEANUP(forward_pose_kin_mod_cleanup)
