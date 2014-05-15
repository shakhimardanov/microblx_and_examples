/*
 * joint_imped_controller microblx function block (autogenerated, don't edit)
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
char joint_imped_controller_meta[] =
	" { doc='',"
	"   real-time=true,"
	"}";

/* declaration of block configuration */
ubx_config_t joint_imped_controller_config[] = {
	{ .name="number_of_axis", .type_name = "int", .doc="" },
	{ .name="stiffness", .type_name = "double", .value = { .len = 5}, .doc="" },
	{ .name="damping", .type_name = "double", .value = { .len = 5}, .doc="" },
	{ NULL },
};

/* declaration port block ports */
ubx_port_t joint_imped_controller_ports[] = {
	{ .name="input_des_joint_poses", .in_type_name="double", .in_data_len=NR_OF_JOINTS, .doc="desired input joint positions from trajectory generator"  },
	{ .name="input_des_joint_rates", .in_type_name="double", .in_data_len=NR_OF_JOINTS, .doc="desired input joint rates from trajectory generator"  },
	{ .name="input_msr_joint_poses", .in_type_name="double", .in_data_len=NR_OF_JOINTS, .doc="measured joint positions"  },
	{ .name="input_msr_joint_rates", .in_type_name="double", .in_data_len=NR_OF_JOINTS, .doc="measured joint rates"  },
	{ .name="output_des_joint_acc", .out_type_name="double", .out_data_len=NR_OF_JOINTS, .doc="desired computed joint acceleration"  },
	{ NULL },
};

/* declare a struct port_cache */
struct joint_imped_controller_port_cache {
	ubx_port_t* input_des_joint_poses;
	ubx_port_t* input_des_joint_rates;
	ubx_port_t* input_msr_joint_poses;
	ubx_port_t* input_msr_joint_rates; //joint twist contribution
	ubx_port_t* output_des_joint_acc;
};

/* declare a helper function to update the port cache this is necessary
 * because the port ptrs can change if ports are dynamically added or
 * removed. This function should hence be called after all
 * initialization is done, i.e. typically in 'start'
 */
static void update_port_cache(ubx_block_t *b, struct joint_imped_controller_port_cache *pc)
{
	pc->input_des_joint_poses = ubx_port_get(b, "input_des_joint_poses");
	pc->input_des_joint_rates = ubx_port_get(b, "input_des_joint_rates");
	pc->input_msr_joint_poses = ubx_port_get(b, "input_msr_joint_poses");
	pc->input_msr_joint_rates = ubx_port_get(b, "input_msr_joint_rates");
	pc->output_des_joint_acc = ubx_port_get(b, "output_des_joint_acc");
}


/* for each port type, declare convenience functions to read/write from ports */
def_read_arr_fun(read_des_joint_poses, double, NR_OF_JOINTS)
def_read_arr_fun(read_des_joint_rates, double, NR_OF_JOINTS)
def_read_arr_fun(read_msr_joint_poses, double, NR_OF_JOINTS)
def_read_arr_fun(read_msr_joint_rates, double, NR_OF_JOINTS)
def_write_arr_fun(write_des_joint_acc, double, NR_OF_JOINTS)

/* block operation forward declarations */
int joint_imped_controller_init(ubx_block_t *b);
int joint_imped_controller_start(ubx_block_t *b);
void joint_imped_controller_stop(ubx_block_t *b);
void joint_imped_controller_cleanup(ubx_block_t *b);
void joint_imped_controller_step(ubx_block_t *b);


/* put everything together */
ubx_block_t joint_imped_controller_block = {
	.name = "joint_imped_controller",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = joint_imped_controller_meta,
	.configs = joint_imped_controller_config,
	.ports = joint_imped_controller_ports,

	/* ops */
	.init = joint_imped_controller_init,
	.start = joint_imped_controller_start,
	.stop = joint_imped_controller_stop,
	.cleanup = joint_imped_controller_cleanup,
	.step = joint_imped_controller_step,
};


/* joint_imped_controller module init and cleanup functions */
int joint_imped_controller_mod_init(ubx_node_info_t* ni)
{
	DBG(" ");
	int ret = -1;
	// ubx_type_t *tptr;

	// for(tptr=types; tptr->name!=NULL; tptr++) {
	// 	if(ubx_type_register(ni, tptr) != 0) {
	// 		goto out;
	// 	}
	// }

	if(ubx_block_register(ni, &joint_imped_controller_block) != 0)
		goto out;

	ret=0;
out:
	return ret;
}

void joint_imped_controller_mod_cleanup(ubx_node_info_t *ni)
{
	DBG(" ");
	// const ubx_type_t *tptr;

	// for(tptr=types; tptr->name!=NULL; tptr++)
	// 	ubx_type_unregister(ni, tptr->name);

	ubx_block_unregister(ni, "joint_imped_controller");
}

/* declare module init and cleanup functions, so that the ubx core can
 * find these when the module is loaded/unloaded */
UBX_MODULE_INIT(joint_imped_controller_mod_init)
UBX_MODULE_CLEANUP(joint_imped_controller_mod_cleanup)