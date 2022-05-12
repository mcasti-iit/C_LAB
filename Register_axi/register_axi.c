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
#include <linux/uaccess.h>
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
#include <linux/delay.h>

#include "ioctl_regs_define.h"


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

/* hardware settings */
#define INT_HANDLE			BIT(27)  // Equivalente a scrivere INT_HANDLE      0x08000000
#define LED_ON 					BIT(26)  // Equivalente a scrivere CLKGEN_HW_RESET 0x04000000
#define CLKGEN_HW_RESET BIT(25)  // Equivalente a scrivere CLKGEN_HW_RESET 0x02000000
#define CLKGEN_ON 			BIT(24)  // Equivalente a scrivere CLKGEN_ON 			 0x01000000

/* magic constants */
#define REG_MAGIC			0x485055

#define REGS_MINOR_COUNT 10


#define REGS_IOCTL_READTIMESTAMP			1

#define REG_LOG 1

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
	// Spazio customizzato
	u32 hw_reg;											// L'immagine dei settings hardware	
	unsigned int irq;								// Interrupts
	spinlock_t irq_lock;						// Spinlock 
};


static struct dentry *regs_debugfsdir = NULL;

static dev_t regs_drv_devt;				// Una tag ottenuta da una combinazione del Major e dal primo numero del Minor; e' specifica per il driver ed e' in comune con tutti i devices gestiti dal driver

static int regs_id = 0; 					// Variabile globale che serve per assegnare il Minor dell'istanza del device


static void register_write(struct regs_priv *priv, u32 val, int offs) 
{
	writel(val, priv->reg_base + offs);
#if REG_LOG
	printk(KERN_INFO "W32 0x%x = 0x%x\n", offs, val);
#endif
}


static u32 register_read(struct regs_priv *priv, int offs)
{
	u32 val;
	val = readl(priv->reg_base + offs);			// Legge il valore del registro all' offset indicato
#if REG_LOG
	printk(KERN_INFO "R32 0x%x == 0x%x\n", offs, val);
#endif
	return val;
}

static irqreturn_t regs_irq_handler(int irq, void *_priv)
{
	struct regs_priv *priv = _priv; // Cast che serve per dire all' handler che il puntatore void _priv 'e un puntatore di tipo regs_priv
	
	spin_lock(&priv->irq_lock);					// Attivo lo spinlock
	register_write(priv, (priv->hw_reg | INT_HANDLE), HARDWARE_REG);
	udelay(1);													// Blocca tutto per 1 microsecondo (NOTA: la udelay effettivamente manda in loop fisso il processore)
	register_write(priv, priv->hw_reg, HARDWARE_REG);
	spin_unlock(&priv->irq_lock);				// Disattivo lo spinlock
	
	dev_info(&priv->pdev->dev, "Interrupt!");	// Stampa il messaggio in dmesg
	return IRQ_HANDLED;
}


static int regs_chardev_open(struct inode *i, struct file *f)
{
	struct regs_priv *priv = 															// il puntatore *priv e' ricavato dando:
														container_of(i->i_cdev,   	// 1. il puntatore ad un campo della struttura 
														struct regs_priv, 					// 2. il tipo di struttura
														regs_cdev);               	// 3. il nome del campo della struttura cui si riferisce il puntatore al punto 1

	f->private_data = priv;																// Mette il puntatore alla lavagnetta, appena recuperato, nel campo private_data di f (questo serve per ritrovare nella read, write e ioctl la giusta lavagnetta)
	dev_info(&priv->pdev->dev, "chardev_open executed");	// Stampa il messaggio in dmesg
	return 0;
  }

static ssize_t regs_chardev_read(struct file *f, char __user *user_buffer, size_t length,	loff_t *offset)
{
	
	const char    string_2br[]    = "WHO I AM:\n";
//	const ssize_t string_2br_size = sizeof(string_2br);    // Lunghezza della stringa comprendente lo 0x00 terminale aggiunto dal C
  const ssize_t string_2br_lenght = strlen(string_2br);  // Lunghezza della stringa SENZA lo 0x00 
	int rolling_length = 0;
	struct regs_priv *priv = f->private_data;  						// Recupera la lavagnetta
	char *entire_string;
	int i;
	int ret;
	
	dev_info(&priv->pdev->dev, "chardev_read executed with length: %lld", (string_2br_lenght - *offset));	// Stampa il messaggio in dmesg
	
	register_read(priv,FPGA_RELEASE_REG);   // Lettura dummy per azzerare il contatore del roling register
  
	// Misura della lunghezza del rolling register
	while ((register_read(priv,ID_REG) & 0x000000FF) != 0x00000003)   // Finche' il byte 0 del registro non 'e 0x03, esegue il loop
		rolling_length++;																								// incremente al variabile che conta la lunghezza della stringa rolling
																																		// Nota: inizia con 0x02 (stx) e finisce con 0x03 (etx), il conteggio contempla la 0x02 ma non la 0x03
	
	entire_string = kmalloc(rolling_length+string_2br_lenght, GFP_KERNEL);						  // Alloca la entire string (nota: la lunghezza della rolling considera una posizione in piu' 
																																										//dovuta al 0x02 iniziale, che ci e' utile per metterci il /n)
	if (entire_string == NULL) {																			// Verifica la buona riuscita
		dev_err(&priv->pdev->dev, "Rolling String memory allocation failed");
		return -ENOMEM;
		}

	memset (entire_string, 'F',rolling_length+string_2br_lenght);
	
	register_read(priv,FPGA_RELEASE_REG);   // Lettura dummy per azzerare di nuovo il contatore del roling register
  
	strcpy(entire_string, string_2br);
	
	register_read(priv,ID_REG); // Per eliminare il 0x02 iniziale;
	
	// Misura della lunghezza del rolling register
	for (i = 0; i <= rolling_length-2; i++)   // Finche' il byte 0 del registro non 'e 0x03, esegue il loop
		entire_string[i+string_2br_lenght] = (char)register_read(priv,ID_REG);
	
	entire_string[string_2br_lenght + rolling_length - 1] = '\n'; 

	
	if (length > (string_2br_lenght + rolling_length - *offset))							   	// Determina la lunghezza del buffer da passare come minimo tra il dato in ingresso e la lunghezza della stringa diminuita sdei caratteri gia' letti (offset)
		length = string_2br_lenght + rolling_length - *offset;
	// length = min(lenght, string_2br_lenght + rolling_length - *offset);			 	// alternativa alla modalita' precedente
	
	// copy_to_user(user_buffer, entire_string, length);				  								// entire_string e' l' indirizzo del primo carattere contenuto nella stringa
  ret = copy_to_user(user_buffer, entire_string + *offset, length);							// entire_string e' l' indirizzo del primo carattere contenuto nella stringa, a cui si somma offset
	if (ret) 
		return -EFAULT; 																															// Verifica la buona riuscita della copy_to_user
	//copy_to_user(user_buffer, &entire_string[*offset], length);	// &entire_string[*offset] e' l' indirizzo del carattere nella posizione [offset] (metodo alternativo alla riga precedente)
	
	kfree (entire_string);			// Libera la memoria occupata da entire_string
	
	*offset = *offset + length;

	return length; 																				// Si ritorna il numero di caratteri letti
}



static ssize_t regs_chardev_write(struct file *f, const char __user *user_buffer, size_t length, loff_t *offset)
{
	struct regs_priv *priv = f->private_data;  																			// Recupera la lavagnetta
	char *buffer_from_user;																													// Definisce il puntatore alla stringa
	u32 conv_value;																																	// Definisci il valore u32 in cui si caricher'a il numero espresso in ASCII
	int ret;																																				// Variabile temporanea di ritorno delle funzioni chiamate
	
	dev_info(&priv->pdev->dev, "chardev_write executed with length: %ld", length);	// Stampa il messaggio in dmesg
	buffer_from_user = kmalloc(length+1, GFP_KERNEL);																// Alloca una porzione di memoria di lunghezza lenght alla posizione indicata dal puntatore buffer_from_user
																																									// NOTA: il +1 serve per aggiungere il rerminatore della stringa ('\0', cioe' 0x00)
	ret = copy_from_user(buffer_from_user, user_buffer, length);							      // Copia il buffer dallo spazio utente allo spazio kernel
	if (ret) 
		return -EFAULT; 																																// Verifica la buona riuscita della copy_from_user
	buffer_from_user[length] = '\0';																								// Aggiungi il carattere '\0' in fondo alla stringa
	msleep(2000);																																	  // attendi 2 secondi		
	dev_info(&priv->pdev->dev, "%s", buffer_from_user );														// Stampa il messaggio in dmesg

	ret = kstrtou32(buffer_from_user, 10, &conv_value);															// Converte in u32 la stringa passata dalla write
	kfree (buffer_from_user);																												// Libera la memoria occupata da buffer_from_user
	
	if (ret) 																																				// Controlla l' esito della kstrou32
	{
		dev_err(&priv->pdev->dev, "String conversion failed");
		return ret;
	}
	
	register_write(priv, conv_value, REG003_REG);
	
	return length;

}

static long regs_chardev_ioctl(struct file *f, unsigned int ioctl_cmd, unsigned long ioctl_arg)
{
	struct regs_priv *priv = f->private_data;
	unsigned long flags;																						// Flagnecessaria alle 'spin_lock_irqsave' e 'spin_unlock_irqrestore' 

	
	if (ioctl_cmd == _IOW(MAGIC_NUMBER, IOCTL_CLKGEN, int)) {  			// Se il comando e' per il CLOCKGEN
		if (ioctl_arg == RESET) {																			// Se l' argomento 'e RESET
			
			spin_lock_irqsave(&priv->irq_lock, flags);									// Attivo lo spinlock
			priv->hw_reg |= CLKGEN_HW_RESET;
			register_write(priv, priv->hw_reg, HARDWARE_REG);
			spin_unlock_irqrestore(&priv->irq_lock, flags);							// Disattivo lo spinlock
			
			usleep_range(1000, 2000);																		// Per attese brevi, dell' ordine del millisecondo, si usa usleep_range che ha una sorta di tolleranza 
			
			spin_lock_irqsave(&priv->irq_lock, flags);									// Attivo lo spinlock
			priv->hw_reg &= ~(CLKGEN_HW_RESET);
			register_write(priv, priv->hw_reg , HARDWARE_REG);
			spin_unlock_irqrestore(&priv->irq_lock, flags);							// Disattivo lo spinlock
			
			dev_info(&priv->pdev->dev, "Clock generator has been RESET");
		}
		else {
			
			spin_lock_irqsave(&priv->irq_lock, flags);									// Attivo lo spinlock
			if (ioctl_arg == ON) {																			// Se l' argomento e' ON 
				priv->hw_reg |= CLKGEN_ON;
			}
			else if (ioctl_arg == OFF) {																// Se l' argomento e' OFF
				priv->hw_reg &= ~(CLKGEN_ON);															// Nota: la ~ opera come inversione bitwise
			}
			else {
				spin_unlock_irqrestore(&priv->irq_lock, flags);						// Disattivo lo spinlock
				dev_err(&priv->pdev->dev, "Wrong argument for GLKGEN\n");  // Nota: ridondante, gia' effettuato nel programma utente per le IOCTL
				return -EINVAL;
			}
			register_write(priv, priv->hw_reg, HARDWARE_REG);
			spin_unlock_irqrestore(&priv->irq_lock, flags);						// Disattivo lo spinlock
			dev_info(&priv->pdev->dev, "Clock Generator is now %s", (ioctl_arg == ON) ? "ON" : "OFF" );			// Stampa il messaggio in dmesg, usando l' operatore ternario
		}
	}
	
	else if (ioctl_cmd == _IOW(MAGIC_NUMBER, IOCTL_LED, int)) {  	// Se il comando e' per il LED	
		
		spin_lock_irqsave(&priv->irq_lock, flags);									// Attivo lo spinlock
		if (ioctl_arg == ON) {																			// Se l' argomento e' ON 
			priv->hw_reg |= LED_ON;
			}
			else if (ioctl_arg == OFF) {															// Se l' argomento e' OFF
			priv->hw_reg &= ~(LED_ON);																// Nota: la ~ opera come inversione bitwise
			}
		else {
			spin_unlock_irqrestore(&priv->irq_lock, flags);						// Disattivo lo spinlock
			dev_err(&priv->pdev->dev, "Wrong argument for LED\n");  	// Nota: ridondante, gia' effettuato nel programma utente per le IOCTL
		return -EINVAL;
		}
		register_write(priv, priv->hw_reg, HARDWARE_REG);
		spin_unlock_irqrestore(&priv->irq_lock, flags);							// Disattivo lo spinlock
		
		dev_info(&priv->pdev->dev, "LED is now %s", (ioctl_arg == ON) ? "ON" : "OFF" );			// Stampa il messaggio in dmesg, usando l' operatore ternario
	}
	
	else {
		dev_err(&priv->pdev->dev, "Wrong IOCTL command\n");  	// Nota: ridondante, gia' effettuato nel programma utente per le IOCTL
		return -EINVAL;
	}
	
	dev_info(&priv->pdev->dev, "chardev_ioctl executed");
	return 0;
}



static struct file_operations regs_fops = {												// Definisce le File Operations
	.owner = THIS_MODULE,	
	.open = regs_chardev_open,
	.read = regs_chardev_read,
	.write= regs_chardev_write,
	.unlocked_ioctl = regs_chardev_ioctl,
//	.release = regs_chardev_close,
};


static int regs_device_probe(struct platform_device *pdev)
{
	struct regs_priv *priv; 																				// Puntatore alla lavagnetta
	struct resource *res;																						// Puntatore ad una struttura delle risorse del device
	struct debugfs_regset32 *regset;																// Puntatore alla struttura del debug dei registri
	char buf[128];
 
	int ret;
		
	priv = kzalloc(sizeof(struct regs_priv), GFP_KERNEL);						// Alloca la lavagnetta azzerandola (kmalloc alloca, kzalloc alloca ed azzera)
	
	spin_lock_init(&priv->irq_lock);																// Inizializza lo spin_lock (la macro vuole il puntatore)
	
	platform_set_drvdata(pdev, priv);																// Associa al platform_device (il device su AXI) la lavagnetta
	priv->pdev = pdev;																							// Salva pdev(la rappresentazine del device memory mapped) nella lavagnetta 
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);						// Trova nel DTS la mappatura della IP
	priv->reg_base = devm_ioremap_resource(&pdev->dev, res);				// Chiede al Kernel di creare in MMU una entry che permetta alla CPU di arrivare all'indirirzzo fisico (e quindi si ha l' accesso ai registri)
	
	register_write(priv, priv->hw_reg , HARDWARE_REG);							// Sincronizza il registro nella FPGA con gli hardware settings presenti nella lavagnetta
	
	priv->irq = platform_get_irq(pdev, 0);													// Associa al campo IRQ della lavagnetta le informazioni dell'interrupt dedicato alla periferica (in arrico dal DTS)
																																	// NOTA: lo '0' dovrebbe essere l' indice, per identificare diversi interrupts
	if (priv->irq < 0) {																						// Verica di buon fine
		dev_err(&pdev->dev, "Error getting irq\n");
		return priv->irq;
	}
	ret =	devm_request_irq(&pdev->dev, priv->irq, regs_irq_handler, // Abilita l' interrupt e gli associa l' handler 'regs_irq_handler' 
				IRQF_SHARED, "register_axi (culo)", priv);								// NOTA: e' la versione 'devm', quindi lo scarico viene automaticamwente gestito alla caduta del device &pdev->dev
	if (ret) {																											// NOTA: l' ultimo parametro e' un puntatore a qualcosa che si desidera il kernel passi alla handler
		dev_err(&pdev->dev, "Error requesting irq: %i\n",
		       ret);
		return ret;
	}
	
	
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

static int regs_device_remove(struct platform_device *pdev)				
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
	.probe = regs_device_probe,						// chiamata per ogni occorrenza di caricamento device
	.remove = regs_device_remove,					// chiamata ad ogni occorrenza di scaricamento device
	.driver = {
		   .name = REGS_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = regs_of_match,
		   },
};

static void __exit regs_module_remove(void)											// Viene chiamata un' unica volta quando si digita "rmmod" 
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

module_init(regs_module_init);			// Funzione chiamata all' "insmod" 
module_exit(regs_module_remove);		// Funzione chiamata al "rmmod" 

MODULE_ALIAS("platform:iit-registers-axi");
MODULE_DESCRIPTION("Register module");
MODULE_AUTHOR("Maurizio Casti <maurizio.casti@iit.it>");
MODULE_AUTHOR("Andrea Merello <andrea.merello@iit.it>");
MODULE_LICENSE("GPL v2");
