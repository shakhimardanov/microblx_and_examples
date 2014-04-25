#define DEBUG 1
#include "forward_pose_kin.hpp"

/* edit and uncomment this:
 * UBX_MODULE_LICENSE_SPDX(GPL-2.0+)
 */

using namespace KDL;


/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct forward_pose_kin_info
{
	Chain* chain;
	Frame* poses;
	JntArray* joint_poses;
	/* add custom block local data here */

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct forward_pose_kin_port_cache ports;
};

/* init */
int forward_pose_kin_init(ubx_block_t *b)
{
	int ret = -1;
	struct forward_pose_kin_info *inf;

	Tree youBotTree;
	if (!kdl_parser::treeFromFile("/opt/ros/groovy/stacks/youbot-ros-pkg/youbot_common/youbot_description/robots/youbot.urdf", youBotTree))
    {
        ERR("Failed to construct kdl tree.\n");
        return false;
    }
	
	/* allocate memory for the block local state */
	if ((inf = (struct forward_pose_kin_info*)calloc(1, sizeof(struct forward_pose_kin_info)))==NULL) {
		ERR("forward_pose_kin: failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}
	b->private_data=inf;
	
	inf->chain = new Chain();
	youBotTree.getChain("arm_link_0","arm_link_5",*(inf->chain));
	inf->joint_poses = new JntArray(NR_OF_JOINTS);
	inf->poses = new Frame();
	update_port_cache(b, &inf->ports);
	ret=0;
out:
	return ret;
}

/* start */
int forward_pose_kin_start(ubx_block_t *b)
{
	/* struct forward_pose_kin_info *inf = (struct forward_pose_kin_info*) b->private_data; */
	int ret = 0;
	return ret;
}

/* stop */
void forward_pose_kin_stop(ubx_block_t *b)
{
	/* struct forward_pose_kin_info *inf = (struct forward_pose_kin_info*) b->private_data; */
}

/* cleanup */
void forward_pose_kin_cleanup(ubx_block_t *b)
{
	free(b->private_data);
}

/* step */
void forward_pose_kin_step(ubx_block_t *b)
{
	double msr_pos[NR_OF_JOINTS];
	Frame ee_pose;
	struct kdl_frame outUBXEEFrame;
	Frame linkTempKDLFrame[NR_OF_JOINTS];
	struct kdl_frame outUBXFrame[NR_OF_JOINTS];
	struct forward_pose_kin_info *inf = (struct forward_pose_kin_info*) b->private_data;

	if(read_joint_poses(inf->ports.input_joint_poses, &msr_pos) > 0) //true
	{
		for(unsigned int i=0; i< NR_OF_JOINTS; i++)
		{
			//compute frame/pose from joint position values
			linkTempKDLFrame[i] = inf->chain->getSegment(i).pose(msr_pos[i]);
			ee_pose = ee_pose*linkTempKDLFrame[i];
			//copy rotation matrix
			for(unsigned int j=0; j<9; j++)
			{
				outUBXFrame[i].M.data[j]=linkTempKDLFrame[i].M.data[j];
			}

			//copy position vector		
			outUBXFrame[i].p.x = linkTempKDLFrame[i].p.x();
			outUBXFrame[i].p.y = linkTempKDLFrame[i].p.y();
			outUBXFrame[i].p.z = linkTempKDLFrame[i].p.z();
		
		}
		//copy ee pose to ubx format
		outUBXEEFrame.p.x = ee_pose.p.x();
		outUBXEEFrame.p.y = ee_pose.p.y();
		outUBXEEFrame.p.z = ee_pose.p.z();
		for(unsigned int j=0; j<9; j++)
		{
			outUBXEEFrame.M.data[j]=ee_pose.M.data[j];
		}

		//send out data;
		write_link_poses(inf->ports.output_link_poses, &outUBXFrame);
		write_ee_pose(inf->ports.output_ee_pose, &outUBXEEFrame);
	}
	
}

