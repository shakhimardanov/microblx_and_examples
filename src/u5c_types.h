/*
 * u5c type a function definitions.
 *
 * This file is luajit-ffi parsable, the rest goes into u5c.h
 */

struct u5c_type;
struct u5c_data;
struct u5c_block;

/* serialization */
typedef struct u5c_serialization {
	const char* name;		/* serialization name */
	const char* type;		/* serialization type */\

	int(*serialize)(struct u5c_data*, char* buffer, uint32_t max_size);
	int(*deserialize)(void*, struct u5c_data*);
	UT_hash_handle hh;
} u5c_serialization_t;


/* type and value (data) */

enum {
	TYPE_CLASS_BASIC=1,
	TYPE_CLASS_STRUCT,	/* simple sequential memory struct */
	TYPE_CLASS_CUSTOM	/* requires custom serialization */
};

typedef struct u5c_type {
	const char* name;		/* name: dir/header.h/struct foo*/
	uint32_t type_class;		/* CLASS_STRUCT=1, CLASS_CUSTOM, CLASS_FOO ... */
	unsigned long size;		/* size in bytes */
	void* private_data;		/* private data. */
	u5c_serialization_t* serializations;
	UT_hash_handle hh;
} u5c_type_t;


typedef struct u5c_data {
	const u5c_type_t* type;	/* link to u5c_type */
	unsigned long len;	/* if length> 1 then length of array, else ignored */
	void* data;		/* buffer with size (type->size * length) */
} u5c_data_t;


/* Port attributes */
enum {
	PORT_DIR_IN 	= 1 << 0,
	PORT_DIR_OUT 	= 1 << 1,
};

/* Port state */
enum {
	PORT_ACTIVE 	= 1 << 0,
};


/* return values */
enum {
	PORT_READ_NODATA   	= -1,
	PORT_READ_NEWDATA  	= -2,

	/* ERROR conditions */
	EPORT_INVALID       	= -3,
	EPORT_INVALID_TYPE  	= -4,

	/* Registration, etc */
	EINVALID_BLOCK_TYPE 	= -5,
	ENOSUCHBLOCK        	= -6,
	EALREADY_REGISTERED	= -7,
	EOUTOFMEM 		= -8,
};

/* Port
h * no distinction between type and value
 */
typedef struct u5c_port {
	char* name;		/* name of port */
	char* meta_data;		/* doc, etc. */

	uint32_t attrs;			/* FP_DIR_IN or FP_DIR_OUT */
	uint32_t state;			/* active/inactive */

	char* in_type_name;	/* string data type name */
	char* out_type_name;	/* string data type name */

	u5c_type_t* in_type;		/* resolved in automatically */
	u5c_type_t* out_type;	 	/* resolved in automatically */

	unsigned long in_data_len;	/* max array size of in/out data */
	unsigned long out_data_len;

	struct u5c_block** in_interaction;
	struct u5c_block** out_interaction;

	/* statistics */
	unsigned long stat_writes;
	unsigned long stat_reades;

	/* todo time stats */
} u5c_port_t;


/*
 * u5c configuration
 */
typedef struct u5c_config {
	char* name;
	char* type_name;
	u5c_data_t value;
} u5c_config_t;

/*
 * u5c block
 */

/* Block types */
enum {
	BLOCK_TYPE_COMPUTATION=1,
	BLOCK_TYPE_INTERACTION,
	BLOCK_TYPE_TRIGGER,
};

enum {
	BLOCK_STATE_PREINIT,
	BLOCK_STATE_INACTIVE,
	BLOCK_STATE_ACTIVE,
};

/* block definition */
typedef struct u5c_block {
	char* name;		/* type name */
	char* meta_data;	/* doc, etc. */
	uint32_t type;		/* type, (computation, interaction, trigger) */

	u5c_port_t* ports;
	u5c_config_t* configs;

	int block_state;  /* state of lifecycle */
	char *prototype; /* name of prototype, NULL if none */


	int(*init) (struct u5c_block*);
	int(*start) (struct u5c_block*);
	void(*stop) (struct u5c_block*);
	void(*cleanup) (struct u5c_block*);

	/* type dependent block methods */
	union {
		/* COMP_TYPE_COMPUTATION */
		void(*step) (struct u5c_block*);

		/* COMP_TYPE_INTERACTION */
		struct {
			/* read and write: these are implemented by interactions and
			 * called by the ports read/write */
			int(*read)(struct u5c_block* interaction, u5c_data_t* value);
			void(*write)(struct u5c_block* interaction, u5c_data_t* value);
		};

		/* COMP_TYPE_TRIGGER - no special ops */
		struct {
			int(*add)(struct u5c_block* cblock);
			int(*rm)(const char *name);
		};
	};


	/* statistics, todo step duration */
	unsigned long stat_num_steps;

	void* private_data;

	UT_hash_handle hh;

} u5c_block_t;


/* node information
 * holds references to all known blocks and types
 */
typedef struct u5c_node_info {
	const char *name;

	u5c_block_t *cblocks;	/* known computation blocks */
	u5c_block_t *iblocks;	/* known interaction blocks */
	u5c_block_t *tblocks;	/* known trigger blocks */
	u5c_type_t *types;	/* known types */
} u5c_node_info_t;


/*
 * runtime API
 */

const char* get_typename(u5c_data_t* data);

/*
 * module and node
 */
int __initialize_module(u5c_node_info_t* ni);
void __cleanup_module(u5c_node_info_t* ni);

int u5c_node_init(u5c_node_info_t* ni, const char *name);
void u5c_node_cleanup(u5c_node_info_t* ni);

/*
 * Information/stats
 */
int u5c_num_cblocks(u5c_node_info_t* ni);
int u5c_num_iblocks(u5c_node_info_t* ni);
int u5c_num_tblocks(u5c_node_info_t* ni);
int u5c_num_types(u5c_node_info_t* ni);
int u5c_num_elements(u5c_block_t* blklst);

/*
 * Registration of types
 */
int u5c_block_register(u5c_node_info_t *ni, u5c_block_t* block);
u5c_block_t* u5c_block_unregister(u5c_node_info_t* ni, uint32_t type, const char* name);
int u5c_type_register(u5c_node_info_t* ni, u5c_type_t* type);
u5c_type_t* u5c_type_unregister(u5c_node_info_t* ni, const char* name);

/*
 * create and destroy
 */
u5c_block_t* u5c_block_create(u5c_node_info_t *ni, uint32_t block_type, const char *name, const char *type);
int u5c_block_rm(u5c_node_info_t *ni, uint32_t block_type, const char* name);
u5c_block_t* u5c_block_get(u5c_node_info_t *ni, uint32_t type, const char *name);


/* block life cycle */
int u5c_block_init(u5c_node_info_t* ni, u5c_block_t* b);
int u5c_block_start(u5c_node_info_t* ni, u5c_block_t* b);
int u5c_block_stop(u5c_node_info_t* ni, u5c_block_t* b);
int u5c_block_cleanup(u5c_node_info_t* ni, u5c_block_t* b);

void u5c_cblock_step(u5c_block_t* b);

int u5c_resolve_types(u5c_node_info_t* ni, u5c_block_t* b);
u5c_type_t* u5c_type_get(u5c_node_info_t* ni, const char* name);

/*
 * connecting blocks
 */
int u5c_connect(u5c_port_t* p1, u5c_port_t* p2, u5c_block_t* iblock);
int u5c_connect_one(u5c_port_t* p, u5c_block_t* iblock);

/* intra-block API */
u5c_port_t* u5c_port_get(u5c_block_t* comp, const char *name);
u5c_port_t* u5c_port_add(u5c_block_t* comp, const char *name, const char *type);
u5c_port_t* u5c_port_rm(u5c_block_t* comp);
/* FOR_EACH_INPORT, FOR_EACH_OUTPORT */

u5c_config_t* u5c_config_get(u5c_block_t* b, const char *name);
int u5c_config_set(u5c_block_t* b, const char *name, u5c_data_t* value);
u5c_data_t* u5c_config_get_data(u5c_block_t* b, const char *name);


uint32_t __port_read(u5c_port_t* port, u5c_data_t* res);
void __port_write(u5c_port_t* port, u5c_data_t* res);

u5c_data_t* u5c_alloc_data(u5c_node_info_t *ni, const char* typename, unsigned long array_len);

