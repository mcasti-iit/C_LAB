/*
 *           HeadProcessorUnit (REGSCore) Linux driver.
 *
 * - this version uses scatter-gather (no cyclic) DMA -
 *
 * For streaming engineering test through char interface.
 * May need to be ported to IIO framework exploiting fast iio from
 * analog devics inc.
 *
 * Copyright (c) 2016 Istituto Italiano di Tecnologia
 * Electronic Design Lab.
 *
 */


#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/iopoll.h>
#include <linux/cdev.h>
#include <linux/idr.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/stringify.h>
#include <linux/version.h>


/* names */
#define REGS_NAME "iit-regs"
#define REGS_DRIVER_NAME REGS_NAME"-driver"
#define REGS_CLASS_NAME REGS_NAME"-class"
#define REGS_DEV_NAME REGS_NAME"-dev"
#define REGS_NAME_FMT REGS_NAME"%d"

/* registers */
#define ID_REG 					  0x00
#define FPGA_RELEASE_REG 	0x04
#define HARDWARE_REG 		  0x08
#define REG003_REG 		    0x0C
#define REG004_REG 		    0x10

/* magic constants */
#define REG_MAGIC			0x485055

#define REGS_MINOR_COUNT 10


#define REGS_IOCTL_READTIMESTAMP			1


static struct debugfs_reg32 regs_regs[] = {
	{"ID_REG",									0x00},
	{"FPGA_RELEASE_REG",        0x04},
	{"HARDWARE_REG",						0x08},
	{"REG003_REG",							0x0C},
	{"REG004_REG",							0x10}
};



typedef struct ip_regs {
       u32 reg_offset;
       char rw;
       u32 data;
} ip_regs_t;


struct regs_priv {
	struct platform_device *pdev;		// Puntatore alla platform device
	void __iomem *reg_base;					// Puntatore al primo indirizzo dell'area di memoria virtuale (vista cio'e dal lato del uP prima della MMU)
	struct dentry *debugfsdir;			// Puntatore al riferimento alla directory creata dal driver nella probe dedicata al evice cui si riferisce la lavagnetta 
	struct cdev regs_cdev;					// Un cazzurbolo che rappresenta il device a caratteri
	dev_t regs_devt;                // Una tag ottenuta da una combinazione del Major e dal Minor; e' specifica per il device
};


static struct dentry *regs_debugfsdir = NULL;

static dev_t regs_drv_devt;						// Una tag ottenuta da una combinazione del Major e dal primo numero del Minor; e' specifica per il driver ed e' in comune con tutti i devices gestiti dal driver

static int regs_id = 0; 					// Variabile globale che serve per assegnare il Minor dell'istanza del device


// static void reg_reg_write(struct regs_priv *priv, u32 val, int offs)
// {
// 	writel(val, priv->reg_base + offs);
// 	printk(KERN_INFO "W32 0x%x = 0x%x\n", offs, val);
// }
// 
// static u32 reg_reg_read(struct regs_priv *priv, int offs)
// {
// 	u32 val;
// 	val = readl(priv->reg_base + offs);
// 	printk(KERN_INFO "R32 0x%x == 0x%x\n", offs, val);
// 	return val;
// }

static int regs_chardev_open(struct inode *i, struct file *f)
{
	int ret = 0;
	u32 reg;
	struct regs_priv *priv = 															// il puntatore *priv e' ricavato dando:
														container_of(i->i_cdev,   	// 1. il puntatore ad un campo della struttura 
														struct regs_priv, 					// 2. il tipo di struttura
														regs_cdev);               	// 3. il nome del campo della struttura cui si riferisce il puntatore al punto 1

	f->private_data = priv;																// Mette il puntatore alla lavagnetta, appena recuperato, nel campo private_data di f (questo serve per ritrovare nella read, write e ioctl la giusta lavagnetta)
	dev_info(&priv->pdev->dev, "chardev_open executed");	// Se va a buon fine stampa il messaggio
	return 0;
  }


static struct file_operations regs_fops = {												// Definisce le File Operations
	.owner = THIS_MODULE,	
	.open = regs_chardev_open,
	
//	.read = regs_chardev_read,
//	.write= regs_chardev_write,
//	.release = regs_chardev_close,
//	.unlocked_ioctl = regs_ioctl,
};


static int regs_probe(struct platform_device *pdev)
{
	struct regs_priv *priv; 																				// Puntatore alla lavagnetta
	struct resource *res;																						// Puntatore ad una struttura delle risorse del device
	struct debugfs_regset32 *regset;																// Puntatore alla struttura del debug dei registri
	unsigned int result;
	u32 ver, tmp;
	char buf[128];
 
	int ret;
		
	priv = kmalloc(sizeof(struct regs_priv), GFP_KERNEL);						// Alloca la lavagnetta
	platform_set_drvdata(pdev, priv);																// Associa al platform_device (il device su AXI) la lavagnetta
	priv->pdev = pdev;																							// Salva pdev(la rappresentazine del device memory mapped) nella lavagnetta 
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);						// Trova nel DTS la mappatura della IP
	priv->reg_base = devm_ioremap_resource(&pdev->dev, res);				// Chiede al Kernel di creare in MMU una entry che permetta alla CPU di arrivare all'indirirzzo fisico
	
	if (regs_debugfsdir) {																					// Genera una sottodirectory (con il nome del puntatore) nella cartella regs per ogni istanza del driver caricata
		sprintf(buf, "regs.%pa", &res->start);
		priv->debugfsdir = debugfs_create_dir(buf, regs_debugfsdir);
	}

	if (priv->debugfsdir) {																							// Controlla sel "priv->debugfsdir = debugfs_create_dir(buf, regs_debugfsdir);" ha avuto buon fine
		regset = devm_kzalloc(&pdev->dev, sizeof(*regset), GFP_KERNEL);   // in caso affermativo, si alloca una struttura di tipo "debugfs_regset32" 
		if (!regset)
			return 0;
		regset->regs = regs_regs;																					// regset viene riempita
		regset->nregs = ARRAY_SIZE(regs_regs);
		regset->base = priv->reg_base;
		debugfs_create_regset32("regdump", 0444, priv->debugfsdir, regset); // crea il file "regdump" in cui c'e' la regset
	}
	
	cdev_init(&priv->regs_cdev, &regs_fops);												// Inizializza la regs_cdev
	priv->regs_cdev.owner = THIS_MODULE;														// Fa questa cosa che non sappiamo a che cazzo serva, ma serve.
	priv->regs_devt = MKDEV(MAJOR(regs_drv_devt), regs_id);					// Assegna la variabile regs_devt, univoca per l'istanza del device
	regs_id++;																											// Incremente il valore del Minor (modo becero... i valori eventualmente rilasciati non vengono riutilizzati)
	ret = cdev_add(&priv->regs_cdev, priv->regs_devt, 1); 					// Concretizza ed abilita la cdev
	if (ret) {																											// Se non va a buon fine stampa il messaggio di errore ed esce
		dev_err(&priv->pdev->dev, "Cannot add chrdev \n");
		return ret;
	}
	dev_info(&priv->pdev->dev, "Registered device major: %d, minor:%d\n",		// // Se va a buon fine stampa il messaggio con il Major ed il Minor
		MAJOR(priv->regs_devt), MINOR(priv->regs_devt));
	return 0;
}

static int regs_remove(struct platform_device *pdev)
{
	struct regs_priv *priv = platform_get_drvdata(pdev);

	/* FIXME: resource release ! */
	debugfs_remove_recursive(priv->debugfsdir);								// Distrugge la directoy del regset ed anche il file ivi contenuuto
	cdev_del(&priv->regs_cdev);																	  // Deregistra il cdev con le fops e tutte le sue cose (simmetrico a cdev_add fatto nella probe)
	kfree(priv);																							// disintegra la lavagnetta (simmetrico a kmalloc fatto nella probe)
	return 0;
}

static struct of_device_id regs_of_match[] = {
	{.compatible = "xlnx,registers-axi-1.0",},
	{}
};

MODULE_DEVICE_TABLE(of, regs_of_match);

static struct platform_driver regs_platform_driver = {
	.probe = regs_probe,
	.remove = regs_remove,
	.driver = {
		   .name = REGS_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = regs_of_match,
		   },
};

static void __exit regs_module_remove(void)
{
	platform_driver_unregister(&regs_platform_driver);						// deregistra il driver dal kernel
	debugfs_remove_recursive(regs_debugfsdir);										// cancella la directory madre del driver
	
	if (regs_drv_devt) {																					// se regs_drv_devt non 'e zero (quindi se esiste)
		unregister_chrdev_region(regs_drv_devt, REGS_MINOR_COUNT);	// rilascia la memoria ed il Major (con i Minor associati) che aveva allocato con la alloc_chrdev_region
		}
}

static int __init regs_module_init(void)
{
	int ret;
	
	ret = alloc_chrdev_region(&regs_drv_devt, 0, REGS_MINOR_COUNT, REGS_DEV_NAME);  // Per il Kernel: prepara le strutture; 
	   																																					// Per l' utente: assegna un Major e alloca uno spazio di minor (tanti quanti indicati da REGS_MINOR_COUNT) andando a "riempire" devt
	
	regs_debugfsdir = debugfs_create_dir("regs", NULL);			// Crea la directory "regs" al percorso /sys/kernel/debug
	ret = platform_driver_register(&regs_platform_driver);  // Ogni volta che nel DTS si trova una IP di tipo "xlnx,registers-axi-1.0" viene eseguita la probe
	if (ret) {    																					// Fornisce errore in caso non sia riuscita a registrare il driver
		printk(KERN_ALERT "Error registering driver "
		       REGS_DRIVER_NAME " \n");
	}

	return 0;
}

module_init(regs_module_init);
module_exit(regs_module_remove);

MODULE_ALIAS("platform:iit-registers-axi");
MODULE_DESCRIPTION("Register module");
MODULE_AUTHOR("Maurizio Casti <maurizio.casti@iit.it>");
MODULE_AUTHOR("Andrea Merello <andrea.merello@iit.it>");
MODULE_LICENSE("GPL v2");
