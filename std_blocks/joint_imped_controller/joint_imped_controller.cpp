#define DEBUG 1

#include "joint_imped_controller.hpp"

using namespace KDL;
/* edit and uncomment this:
 * UBX_MODULE_LICENSE_SPDX(GPL-2.0+)
 */

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct joint_imped_controller_info
{
	JntArray* joint_pose_errs;
	JntArray* joint_rate_errs;
	/* add custom block local data here */

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct joint_imped_controller_port_cache ports;
};

/* init */
int joint_imped_controller_init(ubx_block_t *b)
{
	int ret = -1;
	struct joint_imped_controller_info *inf;

	/* allocate memory for the block local state */
	if ((inf = (struct joint_imped_controller_info*)calloc(1, sizeof(struct joint_imped_controller_info)))==NULL) {
		ERR("forward_pose_kin: failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}
	b->private_data=inf;
	
	inf->joint_pose_errs = new JntArray(NR_OF_JOINTS);
	inf->joint_rate_errs = new JntArray(NR_OF_JOINTS);
	update_port_cache(b, &inf->ports);
	ret=0;
out:
	return ret;
}

/* start */
int joint_imped_controller_start(ubx_block_t *b)
{
	/* struct joint_imped_controller_info *inf = (struct joint_imped_controller_info*) b->private_data; */
	int ret = 0;
	return ret;
}

/* stop */
void joint_imped_controller_stop(ubx_block_t *b)
{
	/* struct joint_imped_controller_info *inf = (struct joint_imped_controller_info*) b->private_data; */
}

/* cleanup */
void joint_imped_controller_cleanup(ubx_block_t *b)
{
	free(b->private_data);
}

/* step */
void joint_imped_controller_step(ubx_block_t *b)
{
	struct joint_imped_controller_info *inf = (struct joint_imped_controller_info*) b->private_data;

	double desiredJointPoses[NR_OF_JOINTS] = {0,0,0,0,0};
	double desiredJointRates[NR_OF_JOINTS] = {0,0,0,0,0};
	double measuredJointPoses[NR_OF_JOINTS] = {0,0,0,0,0};
	double measuredJointRates[NR_OF_JOINTS] = {0,0,0,0,0};
	double computedJointAcc[NR_OF_JOINTS] = {0,0,0,0,0};

	unsigned int len;
	double* stiffness = (double*)ubx_config_get_data_ptr(b, "stiffness", &len);
	double* damping = (double*)ubx_config_get_data_ptr(b, "damping", &len);

	if(	
		(read_des_joint_poses(inf->ports.input_des_joint_poses, &desiredJointPoses) > 0) && 
		(read_des_joint_rates(inf->ports.input_des_joint_rates, &desiredJointRates) > 0 ) &&
		(read_msr_joint_poses(inf->ports.input_msr_joint_poses, &measuredJointPoses) > 0)
		&& (read_msr_joint_rates(inf->ports.input_msr_joint_rates, &measuredJointRates) > 0 )
		)
	{
		for(unsigned int i=0; i<NR_OF_JOINTS; i++)
		{
			computedJointAcc[i] = stiffness[i]*(desiredJointPoses[i] - measuredJointPoses[i]) + damping[i] * (desiredJointRates[i] - measuredJointRates[i]);
		}

		write_des_joint_acc(inf->ports.output_des_joint_acc, &computedJointAcc);

	}
}


