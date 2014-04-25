#define DEBUG 1

#include "wrenchprojection.hpp"

using namespace KDL;
/* edit and uncomment this:
 * UBX_MODULE_LICENSE_SPDX(GPL-2.0+)
 */

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct wrenchprojection_info
{
	Chain* chain;
	Frame* poses;
	JntArray* joint_poses;
	/* add custom block local data here */

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct wrenchprojection_port_cache ports;
};

/* init */
int wrenchprojection_init(ubx_block_t *b)
{
	int ret = -1;
	struct wrenchprojection_info *inf;

	Tree youBotTree;
	if (!kdl_parser::treeFromFile("/opt/ros/groovy/stacks/youbot-ros-pkg/youbot_common/youbot_description/robots/youbot.urdf", youBotTree))
    {
        ERR("Failed to construct kdl tree.\n");
        return false;
    }
	
	/* allocate memory for the block local state */
	if ((inf = (struct wrenchprojection_info*)calloc(1, sizeof(struct wrenchprojection_info)))==NULL) {
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
int wrenchprojection_start(ubx_block_t *b)
{
	/* struct wrenchprojection_info *inf = (struct wrenchprojection_info*) b->private_data; */
	int ret = 0;
	return ret;
}

/* stop */
void wrenchprojection_stop(ubx_block_t *b)
{
	/* struct wrenchprojection_info *inf = (struct wrenchprojection_info*) b->private_data; */
}

/* cleanup */
void wrenchprojection_cleanup(ubx_block_t *b)
{
	free(b->private_data);
}

/* step */
void wrenchprojection_step(ubx_block_t *b)
{
	struct wrenchprojection_info *inf = (struct wrenchprojection_info*) b->private_data;

	struct kdl_twist updatedLinkAccTwists[NR_OF_JOINTS];
	struct kdl_twist updatedLinkTwists[NR_OF_JOINTS];
	struct kdl_twist updatedLinkUnitTwists[NR_OF_JOINTS];
	struct kdl_frame updatedLinkPoses[NR_OF_JOINTS];
	struct kdl_frame updatedEEPose;

	KDL::Frame localTempKDLFrame, localTempEEFrame;
	KDL::Twist localTempUnitTwistZ; 
	KDL::Twist localTempTwist;
	KDL::Twist localTempAccTwist;
	KDL::Wrench localTempWrench[NR_OF_JOINTS];
	
	double outUBXJointTorques[NR_OF_JOINTS] = {0,0,0,0,0} ;
	
	if(	(read_ee_pose(inf->ports.input_ee_pose, &updatedEEPose) > 0 )
		&& (read_link_twists(inf->ports.input_link_twists,&updatedLinkTwists) > 0)
		&& (read_link_acctwists(inf->ports.input_link_acctwists, &updatedLinkAccTwists) > 0)
		)
	{
		for(unsigned int i=0; i<NR_OF_JOINTS; i++)
		{
			//copy link twist
			localTempTwist.vel.x(updatedLinkTwists[i].vel.x);
			localTempTwist.vel.y(updatedLinkTwists[i].vel.y);
			localTempTwist.vel.z(updatedLinkTwists[i].vel.z);
			localTempTwist.rot.x(updatedLinkTwists[i].rot.x);
			localTempTwist.rot.y(updatedLinkTwists[i].rot.y);
			localTempTwist.rot.z(updatedLinkTwists[i].rot.z);
			
			//copy link acctwist
			localTempAccTwist.vel.x(updatedLinkAccTwists[i].vel.x);
			localTempAccTwist.vel.y(updatedLinkAccTwists[i].vel.y);
			localTempAccTwist.vel.z(updatedLinkAccTwists[i].vel.z);
			localTempAccTwist.rot.x(updatedLinkAccTwists[i].rot.x);
			localTempAccTwist.rot.y(updatedLinkAccTwists[i].rot.y);
			localTempAccTwist.rot.z(updatedLinkAccTwists[i].rot.z);
            //Compute wrenches on the links which generate accTwists above
	        RigidBodyInertia linkInertia=inf->chain->getSegment(i).getInertia();
            localTempWrench[i]=linkInertia*localTempAccTwist + localTempTwist*(linkInertia*localTempTwist); //-F_total.M.Inverse()*f_ext[i];
			// printf("Computing forces %d      %f  %f   %f   %f   %f   %f\n", i, localTempWrench[i][0], localTempWrench[i][1], localTempWrench[i][2], localTempWrench[i][3], localTempWrench[i][4], localTempWrench[i][5]);
		}
	}	

	if(	(read_link_poses(inf->ports.input_link_poses, &updatedLinkPoses) > 0 )
			&& (read_link_unit_twists(inf->ports.input_link_unit_twists,&updatedLinkUnitTwists) > 0)
	  )
	{
        //Inward sweep
		for(int i=(NR_OF_JOINTS-1); i>=0; i--)
		{
			//copy unit twist
			localTempUnitTwistZ.vel.x(updatedLinkUnitTwists[i].vel.x);
			localTempUnitTwistZ.vel.y(updatedLinkUnitTwists[i].vel.y);
			localTempUnitTwistZ.vel.z(updatedLinkUnitTwists[i].vel.z);
			localTempUnitTwistZ.rot.x(updatedLinkUnitTwists[i].rot.x);
			localTempUnitTwistZ.rot.y(updatedLinkUnitTwists[i].rot.y);
			localTempUnitTwistZ.rot.z(updatedLinkUnitTwists[i].rot.z);

			//copy rotation matrix to KDL rotation format
			for(unsigned int j=0; j<9;j++)
			{
				localTempKDLFrame.M.data[j] = updatedLinkPoses[i].M.data[j];
			}
			//copy position vector to KDL vector format;
			localTempKDLFrame.p.x(updatedLinkPoses[i].p.x);
			localTempKDLFrame.p.y(updatedLinkPoses[i].p.y);
			localTempKDLFrame.p.z(updatedLinkPoses[i].p.z);

			//compute torques by projecting the wrenches to the right subspace
        	outUBXJointTorques[i]=dot(localTempUnitTwistZ,localTempWrench[i]);
        	// printf("Joint torques %d  %f  %f   %f  %f  %f\n",i, outUBXJointTorques[0], outUBXJointTorques[1], outUBXJointTorques[2], outUBXJointTorques[3], outUBXJointTorques[4]);
            if(i!=0)
                localTempWrench[i-1]=localTempWrench[i-1]+localTempKDLFrame*localTempWrench[i];

		}

	}
	write_joint_torques(inf->ports.output_joint_torques, &outUBXJointTorques);

	
}

