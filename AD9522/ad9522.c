#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "ad9522_define.h"

# define SERIAL_PORT_CONFIG_ADDR		0x0000	// RD
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

# define REG_LOG 1

/* max devices that can be handled */
#define AD9522_MINOR_COUNT 1

/* names */
#define AD9522_NAME "iit-AD9522"
#define AD9522_DRIVER_NAME AD9522_NAME"-driver"
#define AD9522_CLASS_NAME AD9522_NAME"-class"
#define AD9522_DEV_NAME AD9522_NAME"-dev"
#define AD9522_NAME_FMT AD9522_NAME"-%d"


static const char    driver_load_string[] = "AD9522 driver module is loaded!\n\0";
static const ssize_t driver_load_string_size = sizeof(driver_load_string);

static struct class  *AD9522_class = NULL;  // Dichiara la struttura class per questo device



static dev_t AD9522_devt;					// Una tag ottenuta da una combinazione del Major e dal primo numero del Minor; 
																	// e' specifica per il driver ed e' in comune con tutti i devices gestiti dal driver

struct AD9522_priv {							// La Lavagnetta
	struct i2c_client *client;			// Puntatore al client I2C
	struct regmap *regmap;					// Puntatore alla struttura regmap
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
		case SERIAL_PORT_CONFIG_ADDR:
		case IO_UPDATE_ADDR:
			return true;
		default:
			return false;
		};
	};

	
static bool AD9522_regmap_volatile(struct device *dev, unsigned int reg)	// Dichiarazione dei registri "volatile" 
{
	switch (reg) {
		case SERIAL_PORT_CONFIG_ADDR:
		case PLL_READBACK_ADDR:
			return true;
		default:
			return false;
		};
	};

static bool AD9522_regmap_writeable(struct device *dev, unsigned int reg)	// Dichiarazione dei registri "writeable" 
{
	switch (reg) {					
		case SERIAL_PORT_CONFIG_ADDR:
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
		case SERIAL_PORT_CONFIG_ADDR:
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
//	{ "AD9522 PLL Clock Generator", 0 },
//        { }
//	};

u16 gtfreq_125_tab [][2]= {		//	Name						-	Note
	{0x0010, 0x7C},							//	PDF_CHARGE_PUMP	-	Deasserts power-down
	{0x0011, 0x08},							//	R_COUNTER_LSB		-	Sets R counter LSB
	{0x0012, 0x00},							//	R_COUNTER_MSB		-	Sets R counter MSB
	{0x0013, 0x00},							//	A_COUNTER				-	Sets A counter
	{0x0014, 0x19},							//	B_COUNTER_LSB		-	Sets B counter LSB
	{0x0015, 0x00},							//	B_CUNTER_MSB		-	Sets B counter MSB
	{0x0016, 0x05},							//	PLL_CTRL_1			-	Sets P prescaler
											
	{0x0018, 0x06},							//	PLL_CTRL_3			-	(Set default value)
	{0x001C, 0x02},							//	PLL_CTRL_7			-	Enable Ref1 clock
	{0x0232, 0x01},							//	IO_UPDATE				-	Apply
											
	{0x0017, 0x00},							//	PLL_CTRL_2			-	(Set default value)
	{0x0018, 0x06},							//	PLL_CTRL_3			-	(Set default value)
	{0x0019, 0x00},							//	PLL_CTRL_4			-	(Set default value)
	{0x001A, 0x00},							//	PLL_CTRL_5			-	(Set default value)
	{0x001B, 0xA0},							//	PLL_CTRL_6			-	Enables freq monitor
	{0x001C, 0x02},							//	PLL_CTRL_7			-	Enables REF1 clock
	{0x001D, 0x80},							//	PLL_CTRL_8			-	(Set default value)
	{0x001E, 0x00},							//	PLL_CTRL_9			-	(Set default value)
											
	{0x01E0, 0x00},							//	VCO_DIVIDER			-	Sets VCO divider
	{0x01E1, 0x02},							//	INPUT_CLKS			-	Selects VCO as source
											
	{0x0018, 0x06},							//	PLL_CTRL_3			-	(Set default value)
	{0x0232, 0x01},							//	IO_UPDATE				-	Applies settings
	{0x0018, 0x07},							//	PLL_CTRL_3			-	Starts VCO calibration
	{0x0232, 0x01},							//	IO_UPDATE				-	Applies settings
											
	{0x0193, 0x44},							//	DIVIDER_1				-	Sets low and high cycles for DIVIDER 1
	{0x0199, 0x44},							//	DIVIDER_3				-	Sets low and high cycles for DIVIDER 3
											
	{0x0018, 0x06},							//	PLL_CTRL_3			-	(Set default value)
	{0x0232, 0x01},							//	IO_UPDATE				-	Applies settings
	{0x0018, 0x07},							//	PLL_CTRL_3			-	Starts VCO calibration
	{0x0232, 0x01},							//	IO_UPDATE				-	Applies settings
};

u16 gtouten_tab [][2]= {		//	Name						-	Note
	{0x00F3, 0x62},							//	OUT3_CONTROL		-	Sets output format for OUT3 and enables it
	{0x00F4, 0x62},							//	OUT4_CONTROL		-	Sets output format for OUT4 and enables it
	{0x00F5, 0x62},							//	OUT5_CONTROL		-	Sets output format for OUT5 and enables it
	{0x00FA, 0xE2},							//	OUT10_CONTROL		-	Sets output format for OUT10 and enables it
											
	{0x0018, 0x06},							//	PLL_CTRL_3			-	(Set default value)
	{0x0232, 0x01},							//	IO_UPDATE				-	Applies settings
	{0x0018, 0x07},							//	PLL_CTRL_3			-	Starts VCO calibration
	{0x0232, 0x01},							//	IO_UPDATE				-	Applies settings
};

u16 spouten_tab [][2]= {			//	Name						-	Note
	{0x00FA, 0xE2},							//	OUT10_CONTROL		-	Sets output format for OUT10 and enables it
											
	{0x0018, 0x06},							//	PLL_CTRL_3			-	(Set default value)
	{0x0232, 0x01},							//	IO_UPDATE				-	Applies settings
	{0x0018, 0x07},							//	PLL_CTRL_3			-	Starts VCO calibration
	{0x0232, 0x01},							//	IO_UPDATE				-	Applies settings
};


static int ad9522_write(struct AD9522_priv *priv, u16 addr, u8 value) 
{
	int ret;
	ret = regmap_write(priv->regmap, (unsigned int)addr, (unsigned int)value);

	if (ret < 0) 																															 													// 	Se non va a buon fine
		dev_err(&priv->client->dev, "Unable to write at address: 0x%03X\n", addr);	                      // 	- stampa il messaggio di errore 		
	else
#if REG_LOG																																														// Altrimenti, se REG_LOG e' diverso da zero,
		dev_info(&priv->client->dev, "Wrote 0x%02x at address 0x%03x\n", value, addr);										// stampa il messaggio di conferma
#endif
	
	return ret;
}


static int ad9522_read(struct AD9522_priv *priv, u16 addr, u8 *value)
{
	int ret;
	unsigned int intvalue;																																							// Serve una variabile di appoggio "unsigned int" 
	ret = regmap_read(priv->regmap, (unsigned int)addr, &intvalue);																			// perche' regmap scrive un unsigned int 
	*value = (u8)intvalue;																																							// che poi viene "castato"  ad u8
	// ret = regmap_read(priv->regmap, (unsigned int)addr, (unsigned int*)value);

	if (ret < 0) 																															 													// 	Se non va a buon fine
		dev_err(&priv->client->dev, "Unable to read at address: 0x%03X\n", addr);	                        // 	- stampa il messaggio di errore 		
	else
#if REG_LOG
		dev_info(&priv->client->dev, "Read 0x%02x at address 0x%03x\n", *value, addr);
#endif

	return ret;
}

static int write_reg_tab(struct AD9522_priv *priv, u16 reg_tab [][2], int len)
{
	int ret;
	int i;
	

	for (i = 0; i < len; i++) 
	{
		ret = ad9522_write(priv, reg_tab[i][0], (u8)reg_tab[i][1]);
		if (ret < 0) {																															 													// 	Se non va a buon fine
			dev_err(&priv->client->dev, "Abort: unable to write at address: 0x%03X\n",reg_tab[i][0]  );					// 	- stampa il messaggio di errore 
   		return ret;																																													// 	- ed esce
		}
	}
	dev_info(&priv->client->dev, "%d Registers wrote", len);																								// Altrimenti stampa il messaggio di conferma
	return ret;
}



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
	int ret ;
	u8  value;


// *** POWDOWN ***	
if (ioctl_cmd == _IO(MAGIC_NUMBER, POWDOWN_CMD)) {
	dev_info(&priv->client->dev, "IOCTL POWDOWN COMMAND has been called\n");
	
	ret = ad9522_write(priv, PDF_CHARGE_PUMP_ADDR, 0x7D);
	
	return ret;	 
}

// ***  POWUP ***	
if (ioctl_cmd == _IO(MAGIC_NUMBER, POWUP_CMD)) {
	dev_info(&priv->client->dev, "IOCTL POWUP COMMAND has been called\n");
	
	ret = ad9522_write(priv, PDF_CHARGE_PUMP_ADDR, 0x7C);

	return ret;	 
}

// ***  RESET ***	
if (ioctl_cmd == _IO(MAGIC_NUMBER, RESET_CMD)) {
	dev_info(&priv->client->dev, "IOCTL RESET COMMAND has been called\n");
	
	ret = ad9522_write(priv, SERIAL_PORT_CONFIG_ADDR, 0x24);

	return ret;	 
}

// *** STATUS ***	
if (ioctl_cmd == _IOR(MAGIC_NUMBER, STATUS_CMD, int*)) {
	dev_info(&priv->client->dev, "IOCTL STATUS COMMAND has been called\n");
	
	ret = ad9522_read(priv, PLL_READBACK_ADDR, &value);
	
	if (ret == 0)
		ret = copy_to_user((void*)ioctl_arg, &value, sizeof(value));
	return ret;	 
}

// *** INIT ***	
else if (ioctl_cmd == _IO(MAGIC_NUMBER, INIT_CMD)) {
	dev_info(&priv->client->dev, "IOCTL INIT COMMAND has been called\n");

	ret = write_reg_tab(priv, gtfreq_125_tab, ARRAY_SIZE(gtfreq_125_tab));
	
	usleep_range(1000, 2000);																		// Per attese brevi, dell' ordine del millisecondo, si usa usleep_range che ha una sorta di tolleranza 


	return ret;
}
 
// *** ENGTCLK ***
else if (ioctl_cmd == _IO(MAGIC_NUMBER, ENGTCLK_CMD)) {
	dev_info(&priv->client->dev, "IOCTL ENGTCLK_CMD COMMAND has been called\n");
	
	ret = write_reg_tab(priv, gtouten_tab, ARRAY_SIZE(gtouten_tab));
	
	return ret;
}	
	
	
// *** ENSPCLK ***
else if (ioctl_cmd == _IO(MAGIC_NUMBER, ENSPCLK_CMD)) {
	dev_info(&priv->client->dev, "IOCTL ENSPCLK_CMD COMMAND has been called\n");

	ret = write_reg_tab(priv, spouten_tab, ARRAY_SIZE(spouten_tab));
	
	return ret;
}
	
	
// 	if (ioctl_cmd == _IOR(MAGIC_NUMBER, IOCTL_CMD_2, int*)) {
// 			printk( KERN_WARNING "Mauri-driver:  IOCTL READ COMMAND has been called\n");	
// 			copy_to_user((void*)ioctl_arg, &priv->ioctl_value, sizeof(int));
// 	}
	
	else {
		dev_err(&priv->client->dev, "Wrong IOCTL command\n");  	// Nota: ridondante, gia' effettuato nel programma utente per le IOCTL
		return -EINVAL;
	}
	
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
	struct device *pret;
	
	cdev_init(&priv->cdev, &AD9522_fops);																					// Inizializza la regs_cdev
	priv->cdev.owner = THIS_MODULE;																								// Fa una cosa che non sappiamo a che serva, ma serve.
	priv->devt = MKDEV(MAJOR(AD9522_devt), minor);																// Assegna la variabile devt, univoca per l'istanza del device
				
	ret = cdev_add(&priv->cdev, priv->devt, 1);																		// Concretizza ed abilita la cdev
	if (ret) {																																		// 	Se non va a buon fine
		dev_err(&priv->client->dev, "Cannot add chrdev \n");												// 	stampa il messaggio di errore 
		return ret;																																	// 	ed esce
	}			
	dev_info(&priv->client->dev, "Registered device major: %d, minor:%d\n",				// 	Se invece va a buon fine stampa il messaggio 
	       MAJOR(priv->devt), MINOR(priv->devt));																	// 	con il Major ed i				
	pret = device_create(AD9522_class, NULL, priv->devt, priv,										// Serve ad una serie di cose, tra cui popolare la sys/class 
		     AD9522_NAME_FMT, minor);																								// con le informazioi del device
	if (IS_ERR(pret)) {																														// 	Se non va a buon fine
		dev_err(&priv->client->dev, "Cannot create entry point of device\n");				// 	- stampa il messaggio di errore 
		cdev_del(&priv->cdev);																											// 	- deregistra il cdev con le fops e tutte le sue cose (simmetrico a cdev_add fatto nella probe)
		return PTR_ERR(pret);																												// 	- ed esce
	}			
				
	minor++;																																			// Incremente il valore del Minor (modo becero... i valori eventualmente rilasciati non vengono riutilizzati)
			
	return 0;			
}			
			
static int AD9522_unregister_chardev(struct AD9522_priv *priv) 			
{			
	device_destroy(AD9522_class, priv->devt);																			// Distrugge il device creato con device_create
	cdev_del(&priv->cdev);																												// Deregistra il cdev con le fops e tutte le sue cose (simmetrico a cdev_add fatto nella probe)
	return 0;
}


static int AD9522_i2c_probe(struct i2c_client *client)
{ 

	int ret;        
	int part_id_reg;
	struct AD9522_priv *priv;
        
	struct regmap *regmap; 
	
	regmap = devm_regmap_init_i2c(client, &AD9522_regmap_config);									// Inizializza in modo "managed" la reg map
	if (IS_ERR(regmap)) {																													// 	Se non va a buon fine
		dev_err(&client->dev, "Unable to init register map");												// 	- stampa il messaggio di errore 
   	return PTR_ERR(regmap);																											// 	- ed esce
	}
	

	
	priv = devm_kmalloc(&client->dev, sizeof(struct AD9522_priv), GFP_KERNEL);		// Alloca in modo "managed" la priv (lavagnetta)
	priv->client = client;																												// - vi appunta il client passato dal kernel
	priv->regmap = regmap;																												// - vi appunta il puntatore alla regmap 
	
	i2c_set_clientdata(client, priv);																							// Associa la priv alla struttura i2c_client		

	ret = regmap_read(priv->regmap, PART_ID_ADDR, &part_id_reg);									//	Verifica la leggibilita' della regmap leggendo un registro
	if (ret < 0) {																																// 	Se non va a buon fine
		dev_err(&client->dev, "Unable to read in register map");										// 	- stampa il messaggio di errore 
   	return ret;																																	// 	- ed esce
	}	              
	
	dev_info(&client->dev, "Found CLOCK GENERATOR device, PART_ID: 0x%04X\n", 		// Stampa il valore del registro letto
					 	part_id_reg);	
	
	ret = AD9522_register_chardev(priv);																					// Registra il device a caratteri
	
	return ret;
			
} 


static int AD9522_i2c_remove(struct i2c_client *client)
{ 
	
	struct AD9522_priv *priv = i2c_get_clientdata(client);
	AD9522_unregister_chardev(priv);
	return 0;

}

// Opzionale
//MODULE_DEVICE_TABLE(i2c, AD9522_id);																						// Definisce dei metadati per il file ko

static struct i2c_driver AD9522_driver = {
        .driver = {
                .name   = "Clock Generator",
                .of_match_table = of_match_ptr(AD9522_of_match),
        },
        .probe_new      = AD9522_i2c_probe,
				.remove         = AD9522_i2c_remove,
//        .id_table       = AD9522_id,
};


static int AD9522_module_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&AD9522_devt, 0, AD9522_MINOR_COUNT, AD9522_DEV_NAME);	// Per il Kernel: prepara le strutture; 
																																										// Per l' utente: assegna un Major e alloca uno spazio di minor 
																																										// (tanti quanti indicati da REGS_MINOR_COUNT) andando a "riempire" devt
	if (ret < 0) {																																		// Verifica di buon fine
		printk(KERN_ALERT "Error allocating chrdev region for driver "									// NOTA: si usa la printk perche' il device non e' ancora stato "tirato su" 
		 	AD9522_DRIVER_NAME " \n");
		ret = -ENOMEM;
		goto exit;
	}
   
  printk( KERN_NOTICE "Registered AD9522 character device with major number = %u\n", MAJOR(AD9522_devt) );
    
	AD9522_class = class_create(THIS_MODULE, AD9522_CLASS_NAME); 	// Crea la classe per il drive; il Kernel crea anche la directory associata
	if (IS_ERR(AD9522_class)) {
		printk(KERN_ALERT "Error creating class " AD9522_CLASS_NAME " \n");
		ret = PTR_ERR(AD9522_class);																// Estrae dal puntatore un codice di errore
		goto unreg_chrreg;
	}	
	
	ret = i2c_add_driver(&AD9522_driver); 
	if (ret < 0) {
		printk(KERN_ALERT "Error registering i2c driver "
		 	AD9522_DRIVER_NAME " \n"); 
		goto unreg_class;
	}	
	
	goto exit;
	
unreg_class:
	class_destroy(AD9522_class);														  		// Distrugge la classe
	
unreg_chrreg: 
	unregister_chrdev_region(AD9522_devt, AD9522_MINOR_COUNT);		// Scarica la chrdev_region
	
exit:		
	return ret;
	
}
    

static void AD9522_module_exit(void)
{
	i2c_del_driver(&AD9522_driver); 															// Scarica il driver
	class_destroy(AD9522_class);														  		// Distrugge la classe
	unregister_chrdev_region(AD9522_devt, AD9522_MINOR_COUNT);		// Scarica la chrdev_region allocata con alloc_chrdev_region
	
	printk( KERN_NOTICE "Unregistered AD9522 character device with major number = %u\n", MAJOR(AD9522_devt) );
	
}
    
module_init(AD9522_module_init);
module_exit(AD9522_module_exit); 





MODULE_AUTHOR("Maurizio Casti <maurizio.casti@iit.it>");
MODULE_DESCRIPTION("Clock Generator Driver");
MODULE_LICENSE("GPL");
                        
