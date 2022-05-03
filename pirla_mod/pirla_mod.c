#include <linux/module.h>

int init_module(void)
{
	int *pippo = NULL;

	printk("Benvenuto, pirla!\n");
	*pippo = 3;
	printk("pippo: %d", *pippo);

	return 0;
}

void cleanup_module(void)

{

	printk("Arrivederci, pirla!\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("mcasti");
MODULE_DESCRIPTION("pirla-mod");
MODULE_VERSION("1.0");
