#define DEBUG 1

#include "cart_imp_cont.hpp"

using namespace KDL;
/* edit and uncomment this:
 * UBX_MODULE_LICENSE_SPDX(GPL-2.0+)
 */

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct cart_imp_controller_info
{
	/* add custom block local data here */

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct cart_imp_controller_port_cache ports;
};

/* init */
int cart_imp_controller_init(ubx_block_t *b)
{
	int ret = -1;
	struct cart_imp_controller_info *inf;
	/* allocate memory for the block local state */
	if ((inf = (struct cart_imp_controller_info*)calloc(1, sizeof(struct cart_imp_controller_info)))==NULL) {
		ERR("forward_pose_kin: failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}
	b->private_data=inf;
	
	update_port_cache(b, &inf->ports);
	ret=0;
out:
	return ret;
}

/* start */
int cart_imp_controller_start(ubx_block_t *b)
{
	/* struct cart_imp_controller_info *inf = (struct cart_imp_controller_info*) b->private_data; */
	int ret = 0;
	return ret;
}

/* stop */
void cart_imp_controller_stop(ubx_block_t *b)
{
	/* struct cart_imp_controller_info *inf = (struct cart_imp_controller_info*) b->private_data; */
}

/* cleanup */
void cart_imp_controller_cleanup(ubx_block_t *b)
{
	free(b->private_data);
}

/* step */
void cart_imp_controller_step(ubx_block_t *b)
{
	struct cart_imp_controller_info *inf = (struct cart_imp_controller_info*) b->private_data;

	struct kdl_twist updatedLinkAccTwists[NR_OF_JOINTS];
	struct kdl_twist updatedLinkTwists[NR_OF_JOINTS];
	struct kdl_twist updatedLinkUnitTwists[NR_OF_JOINTS];
	struct kdl_frame updatedLinkPoses[NR_OF_JOINTS];
	struct kdl_frame updatedEEPose;
	
	double outUBXJointTorques[NR_OF_JOINTS] = {0,0,0,0,0} ;
	
	if(	(read_ee_pose(inf->ports.input_ee_pose, &updatedEEPose) > 0 )
		&& (read_link_twists(inf->ports.input_link_twists,&updatedLinkTwists) > 0)
		&& (read_link_acctwists(inf->ports.input_link_acctwists, &updatedLinkAccTwists) > 0)
		)
	{
		for(unsigned int i=0; i<NR_OF_JOINTS; i++)
		{

		}
	}	

	if(	(read_link_poses(inf->ports.input_link_poses, &updatedLinkPoses) > 0 )
			&& (read_link_unit_twists(inf->ports.input_link_unit_twists,&updatedLinkUnitTwists) > 0)
	  )
	{
        //Inward sweep
		for(int i=(NR_OF_JOINTS-1); i>=0; i--)
		{

		}

	}
	write_joint_torques(inf->ports.output_joint_torques, &outUBXJointTorques);

	
}

