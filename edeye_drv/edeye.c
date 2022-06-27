#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>



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

/* max Eyes that can be handled */
#define EDEYE_MINOR_COUNT 10

/* names */
#define EDEYE_NAME "iit-edeye"
#define EDEYE_DRIVER_NAME EDEYE_NAME"-driver"
#define EDEYE_CLASS_NAME EDEYE_NAME"-class"
#define EDEYE_DEV_NAME EDEYE_NAME"-dev"info@columbiaoptics.com
#define EDEYE_NAME_FMT EDEYE_NAME"%d"

// --------
#define MAGIC_NUMBER 32

#define IOCTL_CMD_1   1
#define IOCTL_CMD_2   2
#define IOCTL_CMD_3   3


static dev_t edeye_devt;
static int minor = 0;

typedef struct {
	int variable_a;
	int variable_b;
} maustruct;


struct edeye_priv {
	struct cdev cdev;
	dev_t devt;
	struct i2c_client *i2c_dev;
	int ioctl_value;
};

static const char    g_s_Hello_World_string[] = "Hello world from the beautyful kernel mode!\n\0";
static const ssize_t g_s_Hello_World_size = sizeof(g_s_Hello_World_string); 




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


static int device_file_open(struct inode *i, struct file *f)
{
	struct edeye_priv *priv = container_of(i->i_cdev,
					     struct edeye_priv, cdev);
	f->private_data = priv;
	return 0;

}


static ssize_t device_file_read(
                        struct file *file_ptr
                       , char __user *user_buffer
                       , size_t count
                       , loff_t *position)
{
	
	if (count > g_s_Hello_World_size) 
		count = g_s_Hello_World_size;
	
	printk( KERN_NOTICE "Simple-driver: Device file is read\n");
	copy_to_user(user_buffer, g_s_Hello_World_string, count);
	return count;
}

static ssize_t device_file_write(
                        struct file *file_ptr
                       , const char __user *user_buffer
                       , size_t count
                       , loff_t *position)
{
	char buf[64];
	if (count > 63)
		count = 63;
	copy_from_user(buf, user_buffer, count);
	buf[count] = '\0';
	printk( KERN_NOTICE "%s", buf);
	return count;
}


static long device_file_ioctl(                           
                        struct file *file_ptr
                       , unsigned int ioctl_cmd
                       , unsigned long ioctl_arg)
{
	
	struct edeye_priv *priv = file_ptr->private_data;
	
	maustruct ioctl_mauval;
	
	if (ioctl_cmd == _IOW(MAGIC_NUMBER, IOCTL_CMD_1, int*)) {
			printk( KERN_WARNING "Mauri-driver:  IOCTL WRITE COMMAND has been called\n");
			copy_from_user(&priv->ioctl_value, (void*)ioctl_arg, sizeof(int));
	}
	
	if (ioctl_cmd == _IOR(MAGIC_NUMBER, IOCTL_CMD_2, int*)) {
			printk( KERN_WARNING "Mauri-driver:  IOCTL READ COMMAND has been called\n");	
			copy_to_user((void*)ioctl_arg, &priv->ioctl_value, sizeof(int));
	}
	
	if (ioctl_cmd == _IOW(MAGIC_NUMBER, IOCTL_CMD_3, maustruct*)) {
			printk( KERN_WARNING "Mauri-driver:  IOCTL MAU COMMAND has been called\n");
			copy_from_user(&ioctl_mauval, (void*)ioctl_arg, sizeof(maustruct));
		  printk( KERN_WARNING "Mauri-driver:  Values are: %d and %d\n", ioctl_mauval.variable_a,ioctl_mauval.variable_b);
	}
	
	return 0;
}




static struct file_operations edeye_fops = 
{
    .owner   				= THIS_MODULE,
	  .open           = device_file_open,
		.read    				= device_file_read,
		.write   				= device_file_write,
		.unlocked_ioctl	= device_file_ioctl,
}; 




static int edeye_register_chardev(struct edeye_priv *priv)
{
	int ret;
	
	cdev_init(&priv->cdev, &edeye_fops);
	priv->cdev.owner = THIS_MODULE;
	priv->devt = MKDEV(MAJOR(edeye_devt), minor++);
	
	ret = cdev_add(&priv->cdev, priv->devt, 1);
	if (ret) {
		dev_err(&priv->i2c_dev->dev, "Cannot add chrdev \n");
		return ret;
	}

	dev_info(&priv->i2c_dev->dev, "Registered device major: %d, minor:%d\n",
	       MAJOR(priv->devt), MINOR(priv->devt));

	return 0;
}

static int edeye_unregister_chardev(struct edeye_priv *priv)
{
	cdev_del(&priv->cdev);
	return 0;
}


static int edeye_i2c_probe(struct i2c_client *client)
{ 

				int ret;

        
        int id_reg, id_reg_temp1, id_reg_temp2;
        char iam[64];
	      int iam_temp;
  			int iam_index = 0;
				struct edeye_priv *priv;
        
 				struct regmap *regmap = devm_regmap_init_i2c(client, &edeye_regmap_config);
				priv = devm_kmalloc(&client->dev, sizeof(struct edeye_priv), GFP_KERNEL);
	      priv->i2c_dev = client;
	
	      i2c_set_clientdata(client, priv);
  	
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
        else {
        	dev_info(&client->dev, "Found an unrecongnized device, ID_REG: 0x%04X\n", id_reg);
        	return -ENODEV;
				}
	
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
//        printk("\n");
			
		return edeye_register_chardev(priv);
			
} 


static int edeye_i2c_remove(struct i2c_client *client)
{ 
	
	struct edeye_priv *priv = i2c_get_clientdata(client);
	edeye_unregister_chardev(priv);
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
				.remove         = edeye_i2c_remove,
//        .id_table       = edeye_id,
};


int register_major(void)
{
	int ret;

	ret = alloc_chrdev_region(&edeye_devt, 0, EDEYE_MINOR_COUNT, EDEYE_DEV_NAME);
	if (ret < 0) {
		printk(KERN_ALERT "Error allocating chrdev region for driver "
			EDEYE_DRIVER_NAME " \n");
		return -ENOMEM;
	}
   
    printk( KERN_NOTICE "EDEYE-driver: registered EDEYE character device with major number = %u\n", MAJOR(edeye_devt) );
    return 0;
}

void unregister_major(void)
{
	unregister_chrdev_region(edeye_devt, EDEYE_MINOR_COUNT);
	printk( KERN_NOTICE "EDEYE-driver: unregistered EDEYE character device with major number = %u\n", MAJOR(edeye_devt) );
} 


static int mauri_mod_init(void)
{
	register_major();
	return i2c_add_driver(&edeye_driver); 
}
    
static void mauri_mod_exit(void)
{
	i2c_del_driver(&edeye_driver); 
	unregister_major();
	
}
    
module_init(mauri_mod_init);
module_exit(mauri_mod_exit); 





MODULE_AUTHOR("Maurizio Casti <maurizio.casti@iit.it>");
MODULE_DESCRIPTION("Event Driven Eye driver");
MODULE_LICENSE("GPL");
                        
