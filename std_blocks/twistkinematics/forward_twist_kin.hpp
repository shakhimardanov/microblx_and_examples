/*
 * forward_twist_kin microblx function block (autogenerated, don't edit)
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
char forward_twist_kin_meta[] =
	" { doc='',"
	"   real-time=true,"
	"}";

/* declaration of block configuration */
ubx_config_t forward_twist_kin_config[] = {
	{ .name="robot_structural_model", .type_name = "char", .doc="file containing urdf model of a robot " },
	{ NULL },
};

/* declaration port block ports */
ubx_port_t forward_twist_kin_ports[] = {
	{ .name="input_joint_poses", .in_type_name="double", .in_data_len=NR_OF_JOINTS, .doc="joint poses coming from a robot"  },
	{ .name="input_joint_twists", .in_type_name="double", .in_data_len=NR_OF_JOINTS, .doc="joint rates coming from a robot"  },
	{ .name="input_link_poses", .in_type_name="struct kdl_frame", .in_data_len=NR_OF_JOINTS, .doc="link poses with respect to a link tip coming from forward pose kinematics"  },
	{ .name="output_link_twists", .out_type_name="struct kdl_twist", .out_data_len=NR_OF_JOINTS, .doc="link twists with respect to a link tip computed by this block"},
	{ .name="output_link_unit_twists", .out_type_name="struct kdl_twist", .out_data_len=NR_OF_JOINTS, .doc="unit twists which define motion subspace of the link and computed by this block"},
	{ .name="output_joint_twists", .out_type_name="struct kdl_twist", .out_data_len=NR_OF_JOINTS, .doc="joint contributions (vj) to a link twist (v) computed by this block"},
	{ .name="input_ee_pose", .in_type_name="struct kdl_frame", .in_data_len=1, .doc="ee/last link pose with respect to the root/base link of a robot"  },
	{ .name="output_ee_twist", .out_type_name="struct kdl_twist", .out_data_len=1, .doc="ee/last link twist with respect to the rooot/base link of a robot"  },
	{ NULL },
};

/* declare a struct port_cache */
struct forward_twist_kin_port_cache {
	ubx_port_t* input_joint_poses;
	ubx_port_t* input_joint_twists;
	ubx_port_t* input_link_poses;
	ubx_port_t* input_ee_pose;
	ubx_port_t* output_link_twists;
	ubx_port_t* output_link_unit_twists;
	ubx_port_t* output_joint_twists;
	ubx_port_t* output_ee_twist;
};

/* declare a helper function to update the port cache this is necessary
 * because the port ptrs can change if ports are dynamically added or
 * removed. This function should hence be called after all
 * initialization is done, i.e. typically in 'start'
 */
static void update_port_cache(ubx_block_t *b, struct forward_twist_kin_port_cache *pc)
{
	pc->input_joint_poses = ubx_port_get(b, "input_joint_poses");
	pc->input_joint_twists = ubx_port_get(b, "input_joint_twists");
	pc->input_link_poses = ubx_port_get(b, "input_link_poses");
	pc->input_ee_pose = ubx_port_get(b,"input_ee_pose");
	pc->output_link_twists = ubx_port_get(b, "output_link_twists");
	pc->output_joint_twists = ubx_port_get(b, "output_joint_twists");
	pc->output_link_unit_twists = ubx_port_get(b, "output_link_unit_twists");
	pc->output_ee_twist = ubx_port_get(b, "output_ee_twist");
}


/* for each port type, declare convenience functions to read/write from ports */
def_read_arr_fun(read_joint_poses, double, NR_OF_JOINTS)
def_read_arr_fun(read_joint_twists, double, NR_OF_JOINTS)
def_read_arr_fun(read_link_poses, struct kdl_frame, NR_OF_JOINTS)
def_write_arr_fun(write_link_twists, struct kdl_twist, NR_OF_JOINTS)
def_write_arr_fun(write_joint_twists, struct kdl_twist, NR_OF_JOINTS)
def_write_arr_fun(write_link_unit_twists, struct kdl_twist, NR_OF_JOINTS)
def_read_fun(read_ee_pose, struct kdl_frame)
def_write_fun(write_ee_twist, struct kdl_twist)

/* block operation forward declarations */
int forward_twist_kin_init(ubx_block_t *b);
int forward_twist_kin_start(ubx_block_t *b);
void forward_twist_kin_stop(ubx_block_t *b);
void forward_twist_kin_cleanup(ubx_block_t *b);
void forward_twist_kin_step(ubx_block_t *b);


/* put everything together */
ubx_block_t forward_twist_kin_block = {
	.name = "forward_twist_kin",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = forward_twist_kin_meta,
	.configs = forward_twist_kin_config,
	.ports = forward_twist_kin_ports,

	/* ops */
	.init = forward_twist_kin_init,
	.start = forward_twist_kin_start,
	.stop = forward_twist_kin_stop,
	.cleanup = forward_twist_kin_cleanup,
	.step = forward_twist_kin_step,
};


/* forward_twist_kin module init and cleanup functions */
int forward_twist_kin_mod_init(ubx_node_info_t* ni)
{
	DBG(" ");
	int ret = -1;
	// ubx_type_t *tptr;

	// for(tptr=types; tptr->name!=NULL; tptr++) {
	// 	if(ubx_type_register(ni, tptr) != 0) {
	// 		goto out;
	// 	}
	// }

	if(ubx_block_register(ni, &forward_twist_kin_block) != 0)
		goto out;

	ret=0;
out:
	return ret;
}

void forward_twist_kin_mod_cleanup(ubx_node_info_t *ni)
{
	DBG(" ");
	// const ubx_type_t *tptr;

	// for(tptr=types; tptr->name!=NULL; tptr++)
	// 	ubx_type_unregister(ni, tptr->name);

	ubx_block_unregister(ni, "forward_twist_kin");
}

/* declare module init and cleanup functions, so that the ubx core can
 * find these when the module is loaded/unloaded */
UBX_MODULE_INIT(forward_twist_kin_mod_init)
UBX_MODULE_CLEANUP(forward_twist_kin_mod_cleanup)
