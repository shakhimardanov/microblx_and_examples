meta_data = [[
{
	doc="velocity based trajectory generator using reflexxes library",
	license="LGPLv3",
}
]]

NUM_DOF=5

return block 
{
      name="rml_vel",
      meta_data=meta_data,
      port_cache=true,

      types = {
	 -- { name="vector", class='struct' }, -- Enum will follow once implemented in C
	 -- { name="robot_data", class='struct' },
      },

      configurations= {
	 { name="max_vel", type_name="double", len=NUM_DOF, doc="maximum velocity" },
      },

      ports = {
	 { name="msr_pos", in_type_name="double", in_data_len=NUM_DOF, doc="current measured position" },
	 { name="msr_vel", in_type_name="double", in_data_len=NUM_DOF, doc="current measured velocity" },
	 { name="des_pos", in_type_name="double", in_data_len=NUM_DOF, doc="desired target position" },

	 { name="cmd_pos", out_type_name="double", out_data_len=NUM_DOF, doc="new position (controller input)" },
	 { name="cmd_vel", out_type_name="double", out_data_len=NUM_DOF, doc="new velocity (controller input)" },
	 { name="cmd_acc", out_type_name="double", out_data_len=NUM_DOF, doc="new acceleration (controller input)" },
      },
      
      operations = { start=true, stop=true, step=true }
}