#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>


# define SILICON_REV_ADDR						0x0002	// RD
# define PART_ID_ADDR								0x0003	// RD
# define PDF_CHARGE_PUMP_ADDR 			0x0010  // WR
# define R_COUNTER_LSB_ADDR 				0x0011  // WR
# define R_COUNTER_MSB_ADDR 				0x0012  // WR
# define A_COUNTER_ADDR 						0x0013  // WR
# define B_COUNTER_LSB_ADDR 				0x0014  // WR
# define B_CUNTER_MSB_ADDR 					0x0015  // WR
# define PLL_CTRL_1_ADDR 						0x0016  // WR
# define PLL_CTRL_2_ADDR 						0x0017  // WR
# define PLL_CTRL_3_ADDR 						0x0018  // WR
# define PLL_CTRL_4_ADDR 						0x0019  // WR
# define PLL_CTRL_5_ADDR 						0x001A  // WR
# define PLL_CTRL_6_ADDR 						0x001B  // WR
# define PLL_CTRL_7_ADDR 						0x001C  // WR
# define PLL_CTRL_8_ADDR 						0x001D  // WR
# define PLL_CTRL_9_ADDR 						0x001E  // WR
# define PLL_READBACK_ADDR 					0x001F	// RD
# define OUT3_CONTROL_ADDR 					0x00F3  // WR
# define OUT4_CONTROL_ADDR 					0x00F4  // WR
# define OUT5_CONTROL_ADDR 					0x00F5  // WR
# define OUT10_CONTROL_ADDR 				0x00FA  // WR
# define DIVIDER_1_ADDR 						0x0193  // WR
# define DIVIDER_3_ADDR 						0x0199  // WR
# define VCO_DIVIDER_ADDR 					0x01E0  // WR
# define INPUT_CLKS_ADDR 						0x01E1  // WR
# define IO_UPDATE_ADDR 						0x0232  // WC


/* max Eyes that can be handled */
#define AD9522_MINOR_COUNT 10

/* names */
#define AD9522_NAME "iit-AD9522"
#define AD9522_DRIVER_NAME AD9522_NAME"-driver"
#define AD9522_CLASS_NAME AD9522_NAME"-class"
#define AD9522_DEV_NAME AD9522_NAME"-dev"
#define AD9522_NAME_FMT AD9522_NAME"%d"

// --------
#define MAGIC_NUMBER 32

#define IOCTL_CMD_1   1
#define IOCTL_CMD_2   2
#define IOCTL_CMD_3   3

static const char    driver_load_string[] = "AD9522 driver module is loaded!\n\0";
static const ssize_t driver_load_string_size = sizeof(driver_load_string);



static dev_t AD9522_devt;					// Una tag ottenuta da una combinazione del Major e dal primo numero del Minor; 
																	// e' specifica per il driver ed e' in comune con tutti i devices gestiti dal driver

struct AD9522_priv {							// La Lavagnetta
	struct i2c_client *client;			// Puntatore al client I2C
	struct cdev cdev;								// Un cazzurbolo che rappresenta il device a caratteri
	dev_t devt;											// Una tag ottenuta da una combinazione del Major e dal Minor; e' specifica per il device
	// Spazio customizzato

};

static int minor = 0;

static const struct of_device_id __maybe_unused AD9522_of_match[] = {
	{ .compatible = "iit.clkgen" },
	{ },
};

	
static bool AD9522_regmap_precious(struct device *dev, unsigned int reg)	// Dichiarazione dei registri "precious" 
{
	switch (reg) {
		case IO_UPDATE_ADDR:
			return true;
		default:
			return false;
		};
	};

	
static bool AD9522_regmap_volatile(struct device *dev, unsigned int reg)	// Dichiarazione dei registri "volatile" 
{
	switch (reg) {
		case PLL_READBACK_ADDR:
			return true;
		default:
			return false;
		};
	};

static bool AD9522_regmap_writeable(struct device *dev, unsigned int reg)	// Dichiarazione dei registri "writeable" 
{
	switch (reg) {
					
		case PDF_CHARGE_PUMP_ADDR:
		case R_COUNTER_LSB_ADDR:
		case R_COUNTER_MSB_ADDR:
		case A_COUNTER_ADDR:
		case B_COUNTER_LSB_ADDR:
		case B_CUNTER_MSB_ADDR:
		case PLL_CTRL_1_ADDR:
		case PLL_CTRL_2_ADDR:
		case PLL_CTRL_3_ADDR:
		case PLL_CTRL_4_ADDR:
		case PLL_CTRL_5_ADDR:
		case PLL_CTRL_6_ADDR:
		case PLL_CTRL_7_ADDR:
		case PLL_CTRL_8_ADDR:
		case PLL_CTRL_9_ADDR:
		case OUT3_CONTROL_ADDR:
		case OUT4_CONTROL_ADDR:
		case OUT5_CONTROL_ADDR:
		case OUT10_CONTROL_ADDR:
		case DIVIDER_1_ADDR:
		case DIVIDER_3_ADDR:
		case VCO_DIVIDER_ADDR:
		case INPUT_CLKS_ADDR:
		case IO_UPDATE_ADDR:
			return true;
		default:
			return false;
		};
	};

static bool AD9522_regmap_readable(struct device *dev, unsigned int reg)	// Dichiarazione dei registri "readable" 
{
	switch (reg) {
		case SILICON_REV_ADDR:
		case PART_ID_ADDR:
		case PDF_CHARGE_PUMP_ADDR:
		case R_COUNTER_LSB_ADDR:
		case R_COUNTER_MSB_ADDR:
		case A_COUNTER_ADDR:
		case B_COUNTER_LSB_ADDR:
		case B_CUNTER_MSB_ADDR:
		case PLL_CTRL_1_ADDR:
		case PLL_CTRL_2_ADDR:
		case PLL_CTRL_3_ADDR:
		case PLL_CTRL_4_ADDR:
		case PLL_CTRL_5_ADDR:
		case PLL_CTRL_6_ADDR:
		case PLL_CTRL_7_ADDR:
		case PLL_CTRL_8_ADDR:
		case PLL_CTRL_9_ADDR:
		case PLL_READBACK_ADDR:
		case OUT3_CONTROL_ADDR:
		case OUT4_CONTROL_ADDR:
		case OUT5_CONTROL_ADDR:
		case OUT10_CONTROL_ADDR:
		case DIVIDER_1_ADDR:
		case DIVIDER_3_ADDR:
		case VCO_DIVIDER_ADDR:
		case INPUT_CLKS_ADDR:
			return true;
		default:
			return false;
		};
	};


	
 static const struct regmap_config AD9522_regmap_config = {	// Configurazine delle mappe registri
	.reg_bits = 16,
	.val_bits = 8,
	.max_register 	= IO_UPDATE_ADDR,
	.readable_reg 	= AD9522_regmap_readable,
	.writeable_reg 	= AD9522_regmap_writeable,
	.precious_reg 	= AD9522_regmap_precious,
	.volatile_reg		= AD9522_regmap_volatile,
	.cache_type			= REGCACHE_RBTREE,
	};
		
				
// Opzionale
//static const struct i2c_device_id AD9522_id[] = {
//	{ "ED-Eye", 0 },
//        { }
//	};

// ************************************************************************************************************************************************************
// FILE OPERATION

// --------------------------------------------------------------------
// La file_operation OPEN

static int device_file_open(struct inode *i, struct file *f)
{
	struct AD9522_priv *priv = 															// il puntatore *priv e' ricavato usando
														container_of(									// la funzione container_of con argomenti:
															i->i_cdev,									//	1. il puntatore ad un campo della struttura 
															struct AD9522_priv, 				// 	2. il tipo di struttura
															cdev);               				// 	3. il nome del campo della struttura cui si riferisce il puntatore al punto 1

	f->private_data = priv;		 															// Mette il puntatore alla lavagnetta, appena recuperato, nel campo private_data di f 
																													// (questo serve per ritrovare nella read, write e ioctl la giusta lavagnetta)
	
	dev_info(&priv->client->dev, "Chardev_open executed");	// Stampa il messaggio in dmesg
	return 0;
 }


// --------------------------------------------------------------------
// La file_operation READ

static ssize_t device_file_read(
                        struct file *f
                       , char __user *user_buffer
                       , size_t count
                       , loff_t *position)
{
	struct AD9522_priv *priv = f->private_data;

	dev_info(&priv->client->dev, "Chardev_read executed");	// Stampa il messaggio in dmesg
	return 0;
}


// --------------------------------------------------------------------
// La file_operation WRITE

static ssize_t device_file_write(
                        struct file *f
                       , const char __user *user_buffer
                       , size_t count
                       , loff_t *position)
{
	struct AD9522_priv *priv = f->private_data;

	dev_info(&priv->client->dev, "Chardev_write executed");	// Stampa il messaggio in dmesg
	return count;
}


// --------------------------------------------------------------------
// La file_operation IOCTL

static long device_file_ioctl(                           
                        struct file *f
                       , unsigned int ioctl_cmd
                       , unsigned long ioctl_arg)
{
	
	struct AD9522_priv *priv = f->private_data;
	

	
// 	if (ioctl_cmd == _IOW(MAGIC_NUMBER, IOCTL_CMD_1, int*)) {
// 			printk( KERN_WARNING "Mauri-driver:  IOCTL WRITE COMMAND has been called\n");
// 			copy_from_user(&priv->ioctl_value, (void*)ioctl_arg, sizeof(int));
// 	}
// 	
// 	if (ioctl_cmd == _IOR(MAGIC_NUMBER, IOCTL_CMD_2, int*)) {
// 			printk( KERN_WARNING "Mauri-driver:  IOCTL READ COMMAND has been called\n");	
// 			copy_to_user((void*)ioctl_arg, &priv->ioctl_value, sizeof(int));
// 	}
	

	
	return 0;
}


// --------------------------------------------------------------------
// La dichiarazione delle file_operation 

static struct file_operations AD9522_fops = 
{
    .owner   				= THIS_MODULE,
	  .open           = device_file_open,
		.read    				= device_file_read,
		.write   				= device_file_write,
		.unlocked_ioctl	= device_file_ioctl,
}; 




static int AD9522_register_chardev(struct AD9522_priv *priv)
{
	int ret;
	
	cdev_init(&priv->cdev, &AD9522_fops);
	priv->cdev.owner = THIS_MODULE;
	priv->devt = MKDEV(MAJOR(AD9522_devt), minor++);	// Assegna la variabile AD9522_devt, univoca per l'istanza del device
	
	ret = cdev_add(&priv->cdev, priv->devt, 1);
	if (ret) {
		dev_err(&priv->client->dev, "Cannot add chrdev \n");
		return ret;
	}

	dev_info(&priv->client->dev, "Registered device major: %d, minor:%d\n",
	       MAJOR(priv->devt), MINOR(priv->devt));

	return 0;
}

static int AD9522_unregister_chardev(struct AD9522_priv *priv)
{
	cdev_del(&priv->cdev);
	return 0;
}


static int AD9522_i2c_probe(struct i2c_client *client)
{ 

	int ret;        
	int part_id_reg;
	struct AD9522_priv *priv;
        
	struct regmap *regmap = devm_regmap_init_i2c(client, &AD9522_regmap_config);
	priv = devm_kmalloc(&client->dev, sizeof(struct AD9522_priv), GFP_KERNEL);
	priv->client = client;
	
	i2c_set_clientdata(client, priv);
  	
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Unable to init register map");
   	return PTR_ERR(regmap);
	}

	ret= regmap_read(regmap, PART_ID_ADDR, &part_id_reg);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to read in register map");
   	return ret;
	}	              
	
	dev_info(&client->dev, "Found CLOCK GENERATOR device, PART_ID: 0x%04X\n", part_id_reg);	
	
		
	return AD9522_register_chardev(priv);
			
} 


static int AD9522_i2c_remove(struct i2c_client *client)
{ 
	
	struct AD9522_priv *priv = i2c_get_clientdata(client);
	AD9522_unregister_chardev(priv);
	return 0;

}

// Opzionale
//MODULE_DEVICE_TABLE(i2c, AD9522_id);

static struct i2c_driver AD9522_driver = {
        .driver = {
                .name   = "Clock Generator",
                .of_match_table = of_match_ptr(AD9522_of_match),
        },
        .probe_new      = AD9522_i2c_probe,
				.remove         = AD9522_i2c_remove,
//        .id_table       = AD9522_id,
};






static int AD8522_drv_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&AD9522_devt, 0, AD9522_MINOR_COUNT, AD9522_DEV_NAME);	// Per il Kernel: prepara le strutture; 
																																										// Per l' utente: assegna un Major e alloca uno spazio di minor 
																																										// (tanti quanti indicati da REGS_MINOR_COUNT) andando a "riempire" devt
	if (ret < 0) {
		printk(KERN_ALERT "Error allocating chrdev region for driver "
		 	AD9522_DRIVER_NAME " \n");
		// dev_err(&client->dev, "Error allocating chrdev region for driver");	
		return -ENOMEM;
	}
   
    printk( KERN_NOTICE "Registered AD9522 character device with major number = %u\n", MAJOR(AD9522_devt) );
		// dev_info(&client->dev, "Registered character device with major number = %u\n", MAJOR(AD9522_devt) );	
    
	
	ret = i2c_add_driver(&AD9522_driver); 
	
	return ret;
	
}
    

static void AD8522_drv_exit(void)
{
	i2c_del_driver(&AD9522_driver); 
	
	unregister_chrdev_region(AD9522_devt, AD9522_MINOR_COUNT);
	printk( KERN_NOTICE "Unregistered AD9522 character device with major number = %u\n", MAJOR(AD9522_devt) );
	// dev_info(&client->dev, "Unregistered character device with major number = %u\n", MAJOR(AD9522_devt) );
	
}
    
module_init(AD8522_drv_init);
module_exit(AD8522_drv_exit); 





MODULE_AUTHOR("Maurizio Casti <maurizio.casti@iit.it>");
MODULE_DESCRIPTION("Clock Generator Driver");
MODULE_LICENSE("GPL");
                        
