#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/regmap.h>

#define WHO_I_AM_ADDR							0x00	// RD
#define ID_LSB_ADDR								0x01	// RD
#define ID_MSB_ADDR								0x02	// RD
#define ID_CUSTOM_ADDR						0x03	// RD
#define FPGA_BETA_REV_ADDR				0x04	// RD
#define FPGA_REV_ADDR							0x05	// RD
#define FPGA_MIN_VER_ADDR					0x06	// RD
#define FPGA_MAJ_VER_ADDR					0x07	// RD
#define HW_VER_ADDR								0x08	// RD
#define HW_CUSTOM_ADDR						0x09	// RD
#define HW_INFO_ADDR							0x0A	// RD
#define HW_SET_ADDR								0x0B	// RW
			
#define I2C_M_SET_ADDR						0x10	// RW
#define I2C_M_CMD_ADDR						0x11	// WC
		
		
#define I2C_M_SLAVE_ADDR_ADDR			0x14	// RW
#define I2C_M_REG_ADDR_ADDR				0x15	// RW
#define I2C_M_DATA_WR_ADDR				0x16	// RW
#define I2C_M_DATA_RD_ADDR				0x17	// RD
	
#define FILTER_BETA_REV_ADDR			0x20	// RD
#define FILTER_REV_ADDR						0x21	// RD
#define FILTER_MIN_VER_ADDR				0x22	// RD
#define FILTER_MAJ_VER_ADDR				0x23	// RD
#define FILTER_CTRL_ADDR					0x24	// RW
#define FILTER_STATUS_ADDR				0x25	// RD
#define ARP_TIMER_TIMEOUT_UP_ADDR	0x26	// RW
#define ARP_TIMER_TIMEOUT_DW_ADDR	0x27	// RW
#define STN_TIMER_TIMEOUT_ADDR		0x28	// RW

#define CORNEXT_BETA_REV_ADDR			0x30	// RD
#define CORNEXT_REV_ADDR					0x31	// RD
#define CORNEXT_MIN_VER_ADDR			0x32	// RD
#define CORNEXT_MAJ_VER_ADDR			0x33	// RD
#define CORNEXT_CTRL_ADDR					0x34	// RW
#define CORNEXT_STATUS_ADDR				0x35	// RD
#define CORNEXT_REPR_THR_ADDR			0x36	// RW

#define CORNEXT_HARRIS_THR_0_ADDR	0x38	// RW
#define CORNEXT_HARRIS_THR_1_ADDR	0x39	// RW
#define CORNEXT_HARRIS_THR_2_ADDR	0x3A	// RW
#define CORNEXT_HARRIS_THR_3_ADDR	0x3B	// RW

#define IMU_CTRL_ADDR							0x44	// RW
#define IMU_STATUS_ADDR						0x45	// RD

#define SYSTEM_STCT_ADDR					0x50	// RW
#define GEN3_PHASE_ALIGN_ADDR			0x51	// RW
#define EVGEN_CTRL_ADDR						0x52	// RW
#define EVGEN_RATE_ADDR						0x53	// RW


static const struct of_device_id __maybe_unused edeye_of_match[] = {
	{ .compatible = "iit.pirlotest" },
	{ },
};

	
static bool edeye_regmap_precious(struct device *dev, unsigned int reg)	
{
	switch (reg) {
		case WHO_I_AM_ADDR:
			return true;
		default:
			return false;
		};
	};

	
static bool edeye_regmap_volatile(struct device *dev, unsigned int reg)	
{
	switch (reg) {
		case WHO_I_AM_ADDR:
			return true;
		default:
			return false;
		};
	};

static bool edeye_regmap_writeable(struct device *dev, unsigned int reg)	
{
	switch (reg) {

		case I2C_M_SET_ADDR: 
		case I2C_M_CMD_ADDR: 
		case I2C_M_SLAVE_ADDR_ADDR: 
		case I2C_M_REG_ADDR_ADDR: 
		case I2C_M_DATA_WR_ADDR: 
//		case FILTER_CTRL_ADDR: 
//		case ARP_TIMER_TIMEOUT_UP_ADDR: 
//		case ARP_TIMER_TIMEOUT_DW_ADDR: 
//		case STN_TIMER_TIMEOUT_ADDR: 
//		case CORNEXT_CTRL_ADDR: 
//		case CORNEXT_REPR_THR_ADDR: 
//		case CORNEXT_HARRIS_THR_0_ADDR: 
//		case CORNEXT_HARRIS_THR_1_ADDR: 
//		case CORNEXT_HARRIS_THR_2_ADDR: 
//		case CORNEXT_HARRIS_THR_3_ADDR: 
		case IMU_CTRL_ADDR: 
		case SYSTEM_STCT_ADDR: 
		case GEN3_PHASE_ALIGN_ADDR: 
		case EVGEN_CTRL_ADDR: 
		case EVGEN_RATE_ADDR: 
			return true;
		default:
			return false;
		};
	};

static bool edeye_regmap_readable(struct device *dev, unsigned int reg)	
{
	switch (reg) {

		case WHO_I_AM_ADDR: 
		case ID_LSB_ADDR: 
		case ID_MSB_ADDR: 
		case ID_CUSTOM_ADDR: 
		case FPGA_BETA_REV_ADDR: 
		case FPGA_REV_ADDR: 
		case FPGA_MIN_VER_ADDR: 
		case FPGA_MAJ_VER_ADDR: 
		case HW_VER_ADDR: 
		case HW_CUSTOM_ADDR: 
		case HW_INFO_ADDR: 
		case HW_SET_ADDR: 
		case I2C_M_SET_ADDR: 
		case I2C_M_CMD_ADDR: 
		case I2C_M_SLAVE_ADDR_ADDR: 
		case I2C_M_REG_ADDR_ADDR: 
		case I2C_M_DATA_WR_ADDR: 
		case I2C_M_DATA_RD_ADDR: 
//		case FILTER_BETA_REV_ADDR: 
//		case FILTER_REV_ADDR: 
//		case FILTER_MIN_VER_ADDR: 
//		case FILTER_MAJ_VER_ADDR: 
//		case FILTER_CTRL_ADDR: 
//		case FILTER_STATUS_ADDR: 
//		case ARP_TIMER_TIMEOUT_UP_ADDR: 
//		case ARP_TIMER_TIMEOUT_DW_ADDR: 
//		case STN_TIMER_TIMEOUT_ADDR: 
//		case CORNEXT_BETA_REV_ADDR: 
//		case CORNEXT_REV_ADDR: 
//		case CORNEXT_MIN_VER_ADDR: 
//		case CORNEXT_MAJ_VER_ADDR: 
//		case CORNEXT_CTRL_ADDR: 
//		case CORNEXT_STATUS_ADDR: 
//		case CORNEXT_REPR_THR_ADDR: 
//		case CORNEXT_HARRIS_THR_0_ADDR: 
//		case CORNEXT_HARRIS_THR_1_ADDR: 
//		case CORNEXT_HARRIS_THR_2_ADDR: 
//		case CORNEXT_HARRIS_THR_3_ADDR: 
		case IMU_CTRL_ADDR: 
		case IMU_STATUS_ADDR: 
		case SYSTEM_STCT_ADDR: 
		case GEN3_PHASE_ALIGN_ADDR: 
		case EVGEN_CTRL_ADDR: 
		case EVGEN_RATE_ADDR: 
			return true;
		default:
			return false;
		};
	};

	
 static const struct regmap_config edeye_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register 	= EVGEN_RATE_ADDR,
	.readable_reg 	= edeye_regmap_readable,
	.writeable_reg 	= edeye_regmap_writeable,
	.precious_reg 	= edeye_regmap_precious,
	.volatile_reg		= edeye_regmap_volatile,
	.cache_type			= REGCACHE_RBTREE,
	};
		
				
// Opzionale
//static const struct i2c_device_id edeye_id[] = {
//	{ "ED-Eye", 0 },
//        { }
//	};

static int edeye_i2c_probe(struct i2c_client *client)
{ 

				int ret;

        
        int id_reg, id_reg_temp1, id_reg_temp2;
        char iam[64];
	      int iam_temp;
        int iam_index = 0;
        
 	struct regmap *regmap = devm_regmap_init_i2c(client, &edeye_regmap_config);
  	
  	if (IS_ERR(regmap)) {
   		dev_err(&client->dev, "Unable to init register map");
   		return PTR_ERR(regmap);
 			}
	
	
	
//      id_reg = (i2c_smbus_read_byte_data(client, 0x02) << 8);
				ret= regmap_read(regmap, 0x02, &id_reg_temp1);
	              
//				id_reg = id_reg + i2c_smbus_read_byte_data(client, 0x01);
	    	ret= regmap_read(regmap, 0x01, &id_reg_temp2);
				id_reg = (id_reg_temp1 << 8) + id_reg_temp2;
	
        if (id_reg == 0x0200) 
        	dev_info(&client->dev, "Found CCAM3_GEN1 device, ID_REG: 0x%04X\n", id_reg);
        else if (id_reg == 0x0220) 
        	dev_info(&client->dev, "Found CCAM3_GEN3 device, ID_REG: 0x%04X\n", id_reg);
        else 
        	dev_info(&client->dev, "Found an unrecongnized device, ID_REG: 0x%04X\n", id_reg);
        	
//        iam[0] = i2c_smbus_read_byte_data(client, 0x00);
				ret= regmap_read(regmap, 0x00, &iam_temp);
        
        do {
//    		iam[iam_index] = i2c_smbus_read_byte_data(client, 0x00);
				ret= regmap_read(regmap, 0x00, &iam_temp);
				iam[iam_index] = iam_temp;
    		iam_index++;
//    		printk(KERN_CONT "%c", iam);
    	} while ( iam[iam_index-1] != 0x03);
    	iam[iam_index-1] = 0x00;
	dev_info(&client->dev,"%s", iam);
//        printk("%s", iam);
        printk("\n");

       
	return 0;
} 

// Opzionale
//MODULE_DEVICE_TABLE(i2c, edeye_id);

static struct i2c_driver edeye_driver = {
        .driver = {
                .name   = "ED-Eye",
                .of_match_table = of_match_ptr(edeye_of_match),
        },
        .probe_new      = edeye_i2c_probe,
//        .id_table       = edeye_id,
};

module_i2c_driver(edeye_driver);

MODULE_AUTHOR("Maurizio Casti <maurizio.casti@iit.it>");
MODULE_DESCRIPTION("Event Driven Eye driver");
MODULE_LICENSE("GPL");
                        
