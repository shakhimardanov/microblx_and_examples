#define DEBUG 1

#include "forward_acctwist_kin.hpp"

using namespace KDL;
/* edit and uncomment this:
 * UBX_MODULE_LICENSE_SPDX(GPL-2.0+)
 */

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct forward_acctwist_kin_info
{
	Chain* chain;
	Frame* poses;
	JntArray* joint_poses;
	/* add custom block local data here */

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct forward_acctwist_kin_port_cache ports;
};

/* init */
int forward_acctwist_kin_init(ubx_block_t *b)
{
	int ret = -1;
	struct forward_acctwist_kin_info *inf;

	Tree youBotTree;
	if (!kdl_parser::treeFromFile("/opt/ros/groovy/stacks/youbot-ros-pkg/youbot_common/youbot_description/robots/youbot.urdf", youBotTree))
    {
        ERR("Failed to construct kdl tree.\n");
        return false;
    }
	
	/* allocate memory for the block local state */
	if ((inf = (struct forward_acctwist_kin_info*)calloc(1, sizeof(struct forward_acctwist_kin_info)))==NULL) {
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
int forward_acctwist_kin_start(ubx_block_t *b)
{
	/* struct forward_acctwist_kin_info *inf = (struct forward_acctwist_kin_info*) b->private_data; */
	int ret = 0;
	return ret;
}

/* stop */
void forward_acctwist_kin_stop(ubx_block_t *b)
{
	/* struct forward_acctwist_kin_info *inf = (struct forward_acctwist_kin_info*) b->private_data; */
}

/* cleanup */
void forward_acctwist_kin_cleanup(ubx_block_t *b)
{
	free(b->private_data);
}

/* step */
void forward_acctwist_kin_step(ubx_block_t *b)
{
	struct forward_acctwist_kin_info *inf = (struct forward_acctwist_kin_info*) b->private_data;

	double desiredJointAcc[NR_OF_JOINTS] = {0,0,0,0,0};
	struct kdl_twist updatedVjTwists[NR_OF_JOINTS];
	struct kdl_twist updatedLinkTwists[NR_OF_JOINTS];
	struct kdl_twist updatedLinkUnitTwists[NR_OF_JOINTS];
	struct kdl_frame updatedLinkPoses[NR_OF_JOINTS];

	KDL::Frame localTempKDLFrame;
	KDL::Twist localTempUnitTwistZ; 
	KDL::Twist localTempVjTwist; 
	KDL::Twist localTempTwist;
	KDL::Twist localTempAccTwist[NR_OF_JOINTS];
	
	struct kdl_twist outUBXAccTwist[NR_OF_JOINTS];
	unsigned int len;
	KDL::Twist *rootAcc;
	rootAcc = (KDL::Twist*) ubx_config_get_data_ptr(b, "root_link_acctwist", &len);
	if(	
		(read_joint_acctwists(inf->ports.input_joint_acctwists, &desiredJointAcc) > 0) && 
		(read_link_poses(inf->ports.input_link_poses, &updatedLinkPoses) > 0 ) &&
		(read_link_twists(inf->ports.input_link_twists,&updatedLinkTwists) > 0) &&
		(read_joint_twists(inf->ports.input_joint_twists, &updatedVjTwists) > 0) &&
		(read_link_unit_twists(inf->ports.input_link_unit_twists,&updatedLinkUnitTwists) > 0)
	 )
	{
		for(unsigned int i=0; i<NR_OF_JOINTS; i++)
		{
			// DBG("In coming pose:\n%3f %3f %3f %3f\n%3f %3f %3f %3f\n%3f %3f %3f %3f\n%3f %3f %3f %3f",
		 //    updatedLinkPoses[i].M.data[0], updatedLinkPoses[i].M.data[1], updatedLinkPoses[i].M.data[2], updatedLinkPoses[i].p.x,
		 //    updatedLinkPoses[i].M.data[3], updatedLinkPoses[i].M.data[4], updatedLinkPoses[i].M.data[5], updatedLinkPoses[i].p.y,
		 //    updatedLinkPoses[i].M.data[6], updatedLinkPoses[i].M.data[7], updatedLinkPoses[i].M.data[8], updatedLinkPoses[i].p.z,
		 //    0.0, 0.0, 0.0, 1.0);
			//copy rotation matrix to KDL rotation format
			for(unsigned int j=0; j<9; j++)
			{
				localTempKDLFrame.M.data[j] = updatedLinkPoses[i].M.data[j];
			}
			//copy position vector to KDL vector format;
			localTempKDLFrame.p.x(updatedLinkPoses[i].p.x);
			localTempKDLFrame.p.y(updatedLinkPoses[i].p.y);
			localTempKDLFrame.p.z(updatedLinkPoses[i].p.z);
			// DBG("Copied poses:\n%3f %3f %3f %3f\n%3f %3f %3f %3f\n%3f %3f %3f %3f\n%3f %3f %3f %3f",
		 //    localTempKDLFrame.M.data[0], localTempKDLFrame.M.data[1], localTempKDLFrame.M.data[2], localTempKDLFrame.p[0],
		 //    localTempKDLFrame.M.data[3], localTempKDLFrame.M.data[4], localTempKDLFrame.M.data[5], localTempKDLFrame.p[1],
		 //    localTempKDLFrame.M.data[6], localTempKDLFrame.M.data[7], localTempKDLFrame.M.data[8], localTempKDLFrame.p[2],
		 //    0.0, 0.0, 0.0, 1.0);
			//copy link twist
			localTempTwist.vel.x(updatedLinkTwists[i].vel.x);
			localTempTwist.vel.y(updatedLinkTwists[i].vel.y);
			localTempTwist.vel.z(updatedLinkTwists[i].vel.z);
			localTempTwist.rot.x(updatedLinkTwists[i].rot.x);
			localTempTwist.rot.y(updatedLinkTwists[i].rot.y);
			localTempTwist.rot.z(updatedLinkTwists[i].rot.z);

			//copy joint twist contribution
			localTempVjTwist.vel.x(updatedVjTwists[i].vel.x);
			localTempVjTwist.vel.y(updatedVjTwists[i].vel.y);
			localTempVjTwist.vel.z(updatedVjTwists[i].vel.z);
			localTempVjTwist.rot.x(updatedVjTwists[i].rot.x);
			localTempVjTwist.rot.y(updatedVjTwists[i].rot.y);
			localTempVjTwist.rot.z(updatedVjTwists[i].rot.z);

			//copy unit twist
			localTempUnitTwistZ.vel.x(updatedLinkUnitTwists[i].vel.x);
			localTempUnitTwistZ.vel.y(updatedLinkUnitTwists[i].vel.y);
			localTempUnitTwistZ.vel.z(updatedLinkUnitTwists[i].vel.z);
			localTempUnitTwistZ.rot.x(updatedLinkUnitTwists[i].rot.x);
			localTempUnitTwistZ.rot.y(updatedLinkUnitTwists[i].rot.y);
			localTempUnitTwistZ.rot.z(updatedLinkUnitTwists[i].rot.z);

			if(i==0)
			{
                localTempAccTwist[i]=localTempKDLFrame.Inverse(*rootAcc)+localTempUnitTwistZ*desiredJointAcc[i]+localTempTwist*localTempVjTwist;
            }
            else
            {
	            localTempAccTwist[i]=localTempKDLFrame.Inverse(localTempAccTwist[i-1])+localTempUnitTwistZ*desiredJointAcc[i]+localTempTwist*localTempVjTwist;
            }
            outUBXAccTwist[i].vel.x = localTempAccTwist[i].vel.x();
            outUBXAccTwist[i].vel.y = localTempAccTwist[i].vel.y();
            outUBXAccTwist[i].vel.z = localTempAccTwist[i].vel.z();
            outUBXAccTwist[i].rot.x = localTempAccTwist[i].rot.x();
            outUBXAccTwist[i].rot.y = localTempAccTwist[i].rot.y();
            outUBXAccTwist[i].rot.z = localTempAccTwist[i].rot.z();
			// printf("Local ACCTwists %d   %f    %f   %f   %f   %f  %f\n", i, localTempAccTwist[i].vel.x(), localTempAccTwist[i].vel.y(), localTempAccTwist[i].vel.z(), localTempAccTwist[i].rot.x(), localTempAccTwist[i].rot.y(), localTempAccTwist[i].rot.z());
		}

		write_link_acctwists(inf->ports.output_link_acctwists, &outUBXAccTwist);

	}
}

