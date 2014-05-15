#define DEBUG 1

#include "forward_twist_kin.hpp"

  // edit and uncomment this:
 UBX_MODULE_LICENSE_SPDX(GPL-2.0+)

using namespace KDL;
/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct forward_twist_kin_info
{
	Chain* chain;
	Frame* poses;
	JntArray* joint_poses;
	/* add custom block local data here */

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct forward_twist_kin_port_cache ports;
};

/* init */
int forward_twist_kin_init(ubx_block_t *b)
{
	int ret = -1;
	struct forward_twist_kin_info *inf;

	Tree youBotTree;
	if (!kdl_parser::treeFromFile("/opt/ros/groovy/stacks/youbot-ros-pkg/youbot_common/youbot_description/robots/youbot.urdf", youBotTree))
    {
        ERR("Failed to construct kdl tree.\n");
        return false;
    }
	
	/* allocate memory for the block local state */
	if ((inf = (struct forward_twist_kin_info*)calloc(1, sizeof(struct forward_twist_kin_info)))==NULL) {
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
int forward_twist_kin_start(ubx_block_t *b)
{
	/* struct forward_twist_kin_info *inf = (struct forward_twist_kin_info*) b->private_data; */
	int ret = 0;
	return ret;
}

/* stop */
void forward_twist_kin_stop(ubx_block_t *b)
{
	/* struct forward_twist_kin_info *inf = (struct forward_twist_kin_info*) b->private_data; */
}

/* cleanup */
void forward_twist_kin_cleanup(ubx_block_t *b)
{
	free(b->private_data);
}

/* step */
void forward_twist_kin_step(ubx_block_t *b)
{
	struct forward_twist_kin_info *inf = (struct forward_twist_kin_info*) b->private_data;

	//these are temporary and used in internal computations/conversions
	KDL::Frame localTempKDLFrame;
	KDL::Twist tempVj, tempZ;
	KDL::Twist eeTwist;
	KDL::Frame eePose;
	KDL::Twist tempLinkLocalTwists[NR_OF_JOINTS];

	// these data come from a robot, for instance
	double msrJointQdot[NR_OF_JOINTS];
	double msrJointQ[NR_OF_JOINTS];
	//these data come from the forward pose kinematics
	struct kdl_frame updatedLinkLocalPoses[NR_OF_JOINTS]; 
	// struct kdl_frame updatedEEPose; 
	//these data go out  from the block
	struct kdl_twist outUBXEETwist;
	struct kdl_twist outUBXVjTwist[NR_OF_JOINTS];
	struct kdl_twist outUBXTwist[NR_OF_JOINTS];
	struct kdl_twist outUBXUnitTwist[NR_OF_JOINTS];
	
	if(	(read_joint_twists(inf->ports.input_joint_twists, &msrJointQdot) > 0)
		&& (read_link_poses(inf->ports.input_link_poses, &updatedLinkLocalPoses) > 0 )
		&& (read_joint_poses(inf->ports.input_joint_poses,&msrJointQ) > 0)
		// && (read_ee_pose(inf->ports.input_ee_pose, &updatedEEPose) > 0)
		)
	{
		for(unsigned int i=0; i<NR_OF_JOINTS; i++)
		{
			//copy rotation matrix to KDL rotation format
			for(unsigned int j=0; j<9;j++)
			{
				localTempKDLFrame.M.data[j] = updatedLinkLocalPoses[i].M.data[j];
			}
			//copy position vector to KDL vector format;
			localTempKDLFrame.p.x(updatedLinkLocalPoses[i].p.x);
			localTempKDLFrame.p.y(updatedLinkLocalPoses[i].p.y);
			localTempKDLFrame.p.z(updatedLinkLocalPoses[i].p.z);

			//unit twist defining motion subspace matrix
			tempZ = localTempKDLFrame.M.Inverse(inf->chain->getSegment(i).twist(msrJointQdot[i], 1.0));

			//copy computed data to microblx understandable type
			outUBXUnitTwist[i].vel.x = tempZ.vel.x();
			outUBXUnitTwist[i].vel.y = tempZ.vel.y();
			outUBXUnitTwist[i].vel.z = tempZ.vel.z();
			outUBXUnitTwist[i].rot.x = tempZ.rot.x();
			outUBXUnitTwist[i].rot.y = tempZ.rot.y();
			outUBXUnitTwist[i].rot.z = tempZ.rot.z();

			//compute joint twist contribution
        	tempVj = localTempKDLFrame.M.Inverse(inf->chain->getSegment(i).twist(msrJointQ[i], msrJointQdot[i]));
			
			//copy computed data to microblx understandable type
			outUBXVjTwist[i].vel.x = tempVj.vel.x();
			outUBXVjTwist[i].vel.y = tempVj.vel.y();
			outUBXVjTwist[i].vel.z = tempVj.vel.z();
			outUBXVjTwist[i].rot.x = tempVj.rot.x();
			outUBXVjTwist[i].rot.y = tempVj.rot.y();
			outUBXVjTwist[i].rot.z = tempVj.rot.z();

			//compute link twist with respect to link tip frame
			if(i == 0)
				tempLinkLocalTwists[i] = tempVj;
			else
				tempLinkLocalTwists[i] = localTempKDLFrame.Inverse(tempLinkLocalTwists[i-1]) + tempVj;
			
			//copy computed data to microblx understandable type
			outUBXTwist[i].vel.x = tempLinkLocalTwists[i].vel.x();
			outUBXTwist[i].vel.y = tempLinkLocalTwists[i].vel.y();
			outUBXTwist[i].vel.z = tempLinkLocalTwists[i].vel.z();
			outUBXTwist[i].rot.x = tempLinkLocalTwists[i].rot.x();
			outUBXTwist[i].rot.y = tempLinkLocalTwists[i].rot.y();
			outUBXTwist[i].rot.z = tempLinkLocalTwists[i].rot.z();
			// printf("Local Twists %d        %f          %f       %f     %f         %f    %f\n", i, tempLinkLocalTwists[i].vel.x(), tempLinkLocalTwists[i].vel.y(), tempLinkLocalTwists[i].vel.z(), tempLinkLocalTwists[i].rot.x(), tempLinkLocalTwists[i].rot.y(), tempLinkLocalTwists[i].rot.z());
		}


	write_link_twists(inf->ports.output_link_twists, &outUBXTwist);
	write_joint_twists(inf->ports.output_joint_twists, &outUBXVjTwist);
	write_link_unit_twists(inf->ports.output_link_unit_twists, &outUBXUnitTwist);
	//EE twist is not computed yet, needs to be done.
	write_ee_twist(inf->ports.output_ee_twist, &outUBXEETwist);
	}
}

