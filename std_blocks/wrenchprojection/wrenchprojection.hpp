/*
 * wrenchprojection microblx function block (autogenerated, don't edit)
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

#define NR_OF_JOINTS 5


/* block meta information */
char wrenchprojection_meta[] =
	" { doc='',"
	"   real-time=true,"
	"}";

/* declaration of block configuration */
ubx_config_t wrenchprojection_config[] = {
	{ .name="robot_structural_model", .type_name = "char", .doc="file containing urdf model of a robot " },
	{ NULL },
};

/* declaration port block ports */
ubx_port_t wrenchprojection_ports[] = {
	{ .name="input_link_poses", .in_type_name="struct kdl_frame", .in_data_len=NR_OF_JOINTS, .doc="link poses with respect to link tip frame coming from forward pose kinematics"  },
	{ .name="input_ee_pose", .in_type_name="struct kdl_frame", .in_data_len=1, .doc="ee link pose with respect to a robot base coming from forward pose kinematics"  },
	{ .name="input_link_twists", .in_type_name="struct kdl_twist", .in_data_len=NR_OF_JOINTS, .doc="link twists with respect to link tip frame coming from forward twist kinematics"  },
	{ .name="input_link_unit_twists", .in_type_name="struct kdl_twist", .in_data_len=NR_OF_JOINTS, .doc="unit twist defining motino subspace of the link"  },
	{ .name="input_link_acctwists", .in_type_name="struct kdl_twist", .in_data_len=NR_OF_JOINTS, .doc="link accelleration twists with respect to link tip frame computed by this block"  },
	{ .name="output_joint_torques", .out_type_name="double", .out_data_len=NR_OF_JOINTS, .doc="torques computed by this block that serve as an input to a robot" },
	{ NULL },
};

/* declare a struct port_cache */
struct wrenchprojection_port_cache {
	ubx_port_t* input_link_poses;
	ubx_port_t* input_ee_pose;
	ubx_port_t* input_link_twists;
	ubx_port_t* input_link_unit_twists;
	ubx_port_t* input_link_acctwists;
	ubx_port_t* output_joint_torques;
};

/* declare a helper function to update the port cache this is necessary
 * because the port ptrs can change if ports are dynamically added or
 * removed. This function should hence be called after all
 * initialization is done, i.e. typically in 'start'
 */
static void update_port_cache(ubx_block_t *b, struct wrenchprojection_port_cache *pc)
{
	pc->input_link_poses = ubx_port_get(b, "input_link_poses");
	pc->input_ee_pose = ubx_port_get(b, "input_ee_pose");
	pc->input_link_twists = ubx_port_get(b, "input_link_twists");
	pc->input_link_unit_twists = ubx_port_get(b, "input_link_unit_twists");
	pc->input_link_acctwists = ubx_port_get(b, "input_link_acctwists");
	pc->output_joint_torques = ubx_port_get(b, "output_joint_torques");
}


/* for each port type, declare convenience functions to read/write from ports */
def_read_arr_fun(read_link_poses, struct kdl_frame, NR_OF_JOINTS)
def_read_fun(read_ee_pose, struct kdl_frame)
def_read_arr_fun(read_link_twists, struct kdl_twist, NR_OF_JOINTS)
def_read_arr_fun(read_link_unit_twists, struct kdl_twist, NR_OF_JOINTS)
def_read_arr_fun(read_link_acctwists, struct kdl_twist, NR_OF_JOINTS)
def_write_arr_fun(write_joint_torques, double, NR_OF_JOINTS)

/* block operation forward declarations */
int wrenchprojection_init(ubx_block_t *b);
int wrenchprojection_start(ubx_block_t *b);
void wrenchprojection_stop(ubx_block_t *b);
void wrenchprojection_cleanup(ubx_block_t *b);
void wrenchprojection_step(ubx_block_t *b);


/* put everything together */
ubx_block_t wrenchprojection_block = {
	.name = "wrenchprojection",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = wrenchprojection_meta,
	.configs = wrenchprojection_config,
	.ports = wrenchprojection_ports,

	/* ops */
	.init = wrenchprojection_init,
	.start = wrenchprojection_start,
	.stop = wrenchprojection_stop,
	.cleanup = wrenchprojection_cleanup,
	.step = wrenchprojection_step,
};


/* wrenchprojection module init and cleanup functions */
int wrenchprojection_mod_init(ubx_node_info_t* ni)
{
	DBG(" ");
	int ret = -1;
	// ubx_type_t *tptr;

	// for(tptr=types; tptr->name!=NULL; tptr++) {
	// 	if(ubx_type_register(ni, tptr) != 0) {
	// 		goto out;
	// 	}
	// }

	if(ubx_block_register(ni, &wrenchprojection_block) != 0)
		goto out;

	ret=0;
out:
	return ret;
}

void wrenchprojection_mod_cleanup(ubx_node_info_t *ni)
{
	DBG(" ");
	// const ubx_type_t *tptr;

	// for(tptr=types; tptr->name!=NULL; tptr++)
	// 	ubx_type_unregister(ni, tptr->name);

	ubx_block_unregister(ni, "wrenchprojection");
}

/* declare module init and cleanup functions, so that the ubx core can
 * find these when the module is loaded/unloaded */
UBX_MODULE_INIT(wrenchprojection_mod_init)
UBX_MODULE_CLEANUP(wrenchprojection_mod_cleanup)
