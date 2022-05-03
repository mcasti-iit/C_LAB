#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define MAGIC_NUMBER 32

#define IOCTL_CMD_1   1
#define IOCTL_CMD_2   2
#define IOCTL_CMD_3   3


static const char    g_s_Hello_World_string[] = "Hello world from the beautyful kernel mode!\n\0";
static const ssize_t g_s_Hello_World_size = sizeof(g_s_Hello_World_string); 

static int ioctl_value;

typedef struct {
	int variable_a;
	int variable_b;
} maustruct;


static ssize_t device_file_read(
                        struct file *file_ptr
                       , char __user *user_buffer
                       , size_t count
                       , loff_t *position)
// {
//     printk( KERN_NOTICE "Simple-driver: Device file is read at offset = %i, read bytes count = %u\n"
//                 , (int)*position
//                 , (unsigned int)count );
//     /* If position is behind the end of a file we have nothing to read */
//     if( *position >= g_s_Hello_World_size )
//         return 0;
//     /* If a user tries to read more than we have, read only as many bytes as we have */
//     if( *position + count > g_s_Hello_World_size )
//         count = g_s_Hello_World_size - *position;
//     if( copy_to_user(user_buffer, g_s_Hello_World_string + *position, count) != 0 )
//         return -EFAULT;    
//     /* Move reading position */
//     *position += count;
//     return count;
// } 
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
	
	maustruct ioctl_mauval;
	
	if (ioctl_cmd == _IOW(MAGIC_NUMBER, IOCTL_CMD_1, int*)) {
			printk( KERN_WARNING "Mauri-driver:  IOCTL WRITE COMMAND has been called\n");
			copy_from_user(&ioctl_value, (void*)ioctl_arg, sizeof(int));
	}
	
	if (ioctl_cmd == _IOR(MAGIC_NUMBER, IOCTL_CMD_2, int*)) {
			printk( KERN_WARNING "Mauri-driver:  IOCTL READ COMMAND has been called\n");	
			copy_to_user((void*)ioctl_arg, &ioctl_value, sizeof(int));
	}
	
	if (ioctl_cmd == _IOW(MAGIC_NUMBER, IOCTL_CMD_3, maustruct*)) {
			printk( KERN_WARNING "Mauri-driver:  IOCTL MAU COMMAND has been called\n");
			copy_from_user(&ioctl_mauval, (void*)ioctl_arg, sizeof(maustruct));
		  printk( KERN_WARNING "Mauri-driver:  Values are: %d and %d\n", ioctl_mauval.variable_a,ioctl_mauval.variable_b);
	}
	
	return 0;
}

			

static struct file_operations mauri_fops = 
{
    .owner   				= THIS_MODULE,
		.read    				= device_file_read,
		.write   				= device_file_write,
		.unlocked_ioctl	= device_file_ioctl,
}; 


static int device_file_major_number = 0;
static const char device_name[] = "Mauri-driver";
int register_device(void)
{
    int result = 0;
    printk( KERN_NOTICE "Mauri-driver: register_device() is called.\n" );
    result = register_chrdev( 0, device_name, &mauri_fops );
    if( result < 0 )
    {
    		printk( KERN_WARNING "Mauri-driver:  can\'t register character device with error code = %i\n", result );
        return result;
    }
    device_file_major_number = result;
    printk( KERN_NOTICE "Mauri-driver: registered character device with major number = %i and minor numbers 0...255\n", device_file_major_number );
    return 0;
}

void unregister_device(void)
{
		printk( KERN_NOTICE "Simple-driver: unregister_device() is called\n" );
    if(device_file_major_number != 0)
    {
     		unregister_chrdev(device_file_major_number, device_name);
			  printk( KERN_NOTICE "Mauri-driver: unregistered character device with major number = %i and minor numbers 0...255\n", device_file_major_number );
    }
} 






static int mauri_mod_init(void)
{
	register_device();
	return  0;
}
    
static void mauri_mod_exit(void)
{
	unregister_device();
	return;
}
    
module_init(mauri_mod_init);
module_exit(mauri_mod_exit); 


MODULE_AUTHOR("Maurizio Casti <maurizio.casti@iit.it>");
MODULE_DESCRIPTION("Character Driver for training");
MODULE_LICENSE("GPL");
