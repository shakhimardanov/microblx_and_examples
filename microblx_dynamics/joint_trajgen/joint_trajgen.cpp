/* 
 * Reflexxes based trajectory generator block.
 */

/* #define DEBUG */
#include "joint_trajgen.hpp"

UBX_MODULE_LICENSE_SPDX(BSD-3-Clause)

/* define a structure for holding the block local state. By assigning an
 * instance of this struct to the block private_data pointer (see init), this
 * information becomes accessible within the hook functions.
 */
struct joint_trajgen_info
{
	int reached;
	double time_duration;
	struct ubx_timespec start_time;
	double* target_joint_pose;
	double** polynomial_coeff;

	/* this is to have fast access to ports for reading and writing, without
	 * needing a hash table lookup */
	struct joint_trajgen_port_cache ports;
};

/* init */
int joint_trajgen_init(ubx_block_t *b)
{
	int ret = -1;
	unsigned int len;
	struct joint_trajgen_info *inf;

	/* allocate memory for the block local state */
	if ((inf = (joint_trajgen_info*) calloc(1, sizeof(struct joint_trajgen_info)))==NULL) {
		ERR("joint_trajgen: failed to alloc memory");
		ret=EOUTOFMEM;
		goto out;
	}

	inf->time_duration = *((double*) ubx_config_get_data_ptr(b, "time_duration", &len));
	inf->target_joint_pose = (double*)ubx_config_get_data_ptr(b, "des_target_pos", &len);

	//polynomial coefficients for each joint traj [NR_OF_JOINTS x POLYNOMIAL_DEGREE]
	inf->polynomial_coeff = new double*[NR_OF_JOINTS];
	for(int i=0; i<NR_OF_JOINTS; i++)
		inf->polynomial_coeff[i] = new double[POLYNOMIAL_DEGREE];
	update_port_cache(b, &inf->ports);
	b->private_data=inf;
	ret=0;
	goto out;

 out:
	return ret;
}

/* start */
int joint_trajgen_start(ubx_block_t *b)
{
	int	ret = 0;
	int sz_msr_pos;
	struct joint_trajgen_info *inf = (struct joint_trajgen_info*) b->private_data;
	for(int i=0; i<NR_OF_JOINTS; i++)
	{
		printf("Start: Target joint poses %d:  %f\n",i, inf->target_joint_pose[i]);
	}
	
	double tmparr[NR_OF_JOINTS];
	sz_msr_pos = read_msr_pos(inf->ports.msr_pos, &tmparr);
	if(sz_msr_pos == 5)
	{
		for(int i=0; i<NR_OF_JOINTS; i++)
		{
			inf->polynomial_coeff[i][0] = tmparr[i];
			inf->polynomial_coeff[i][1] = 0.0;
			inf->polynomial_coeff[i][2] = (inf->target_joint_pose[i] - tmparr[i])*3.0/(inf->time_duration*inf->time_duration);
			inf->polynomial_coeff[i][3] = -(inf->target_joint_pose[i] - tmparr[i])*2.0/(inf->time_duration*inf->time_duration*inf->time_duration);
			
			printf("Polynomial coeff %d:  %f\n",i, inf->polynomial_coeff[i][0] );
			printf("Polynomial coeff %d:  %f\n",i, inf->polynomial_coeff[i][1] );
			printf("Polynomial coeff %d:  %f\n",i, inf->polynomial_coeff[i][2] );
			printf("Polynomial coeff %d:  %f\n",i, inf->polynomial_coeff[i][3] );
		}
	}

	ubx_clock_mono_gettime(&inf->start_time);

	return ret;
}

/* stop */
void joint_trajgen_stop(ubx_block_t *b)
{
	/* struct _info *inf = (struct _info*) b->private_data; */
}

/* cleanup */
void joint_trajgen_cleanup(ubx_block_t *b)
{
	struct joint_trajgen_info *inf;
	inf = (struct joint_trajgen_info*) b->private_data;
	for(int i=0; i < NR_OF_JOINTS; i++)
		free(inf->polynomial_coeff[i]);
	free(inf->polynomial_coeff);
	free(inf);
}

/* step */
void joint_trajgen_step(ubx_block_t *b)
{
	
	double tmparr_pos[NR_OF_JOINTS], tmparr_vel[NR_OF_JOINTS];
	double time_passed;
	struct ubx_timespec now, diff;
	struct joint_trajgen_info *inf = (struct joint_trajgen_info*) b->private_data;

	ubx_clock_mono_gettime(&now);
	ubx_ts_sub(&now, &inf->start_time, &diff);
	time_passed = ubx_ts_to_double(&diff);
	
	if(time_passed > inf->time_duration)
	{
		for(unsigned int i=0; i<NR_OF_JOINTS; i++)
		{
			tmparr_pos[i] = inf->target_joint_pose[i];
			
			tmparr_vel[i] = inf->polynomial_coeff[i][1] + 2*inf->polynomial_coeff[i][2]*inf->time_duration
													   + 3*inf->polynomial_coeff[i][3]*inf->time_duration*inf->time_duration;

		   
		}

	}
	else
	{
		for(unsigned int i=0; i<NR_OF_JOINTS; i++)
		{
			tmparr_pos[i] = inf->polynomial_coeff[i][0] + inf->polynomial_coeff[i][1]*time_passed 
												   + inf->polynomial_coeff[i][2]*time_passed*time_passed
												   + inf->polynomial_coeff[i][3]*time_passed*time_passed*time_passed;
			
			tmparr_vel[i] = inf->polynomial_coeff[i][1] + 2*inf->polynomial_coeff[i][2]*time_passed
													   + 3*inf->polynomial_coeff[i][3]*time_passed*time_passed;

		   
		}
	}


	write_des_pos(inf->ports.des_pos, &tmparr_pos);
	write_des_vel(inf->ports.des_vel, &tmparr_vel);
	



	return;
}
