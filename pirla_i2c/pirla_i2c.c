#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/device.h>

static const struct of_device_id __maybe_unused edeye_of_match[] = {
	{ .compatible = "iit.pirlotest" },
	{ },
};

static const struct i2c_device_id edeye_id[] = {
	{ "ED-Eye", 0 },
        { }
	};

static int edeye_probe(struct i2c_client *client)
{ 
        
        int id_reg;
        char iam[64];
        int iam_index = 0;
        

        id_reg = (i2c_smbus_read_byte_data(client, 0x02) << 8);
        id_reg = id_reg + i2c_smbus_read_byte_data(client, 0x01);
        if (id_reg == 0x0200) 
        	dev_info(&client->dev, "Found CCAM3_GEN1 device, ID_REG: 0x%04X\n", id_reg);
        else if (id_reg == 0x0220) 
        	dev_info(&client->dev, "Found CCAM3_GEN3 device, ID_REG: 0x%04X\n", id_reg);
        else 
        	dev_info(&client->dev, "Found an unrecongnized device, ID_REG: 0x%04X\n", id_reg);
        	
        iam[0] = i2c_smbus_read_byte_data(client, 0x00);
        
        do {
    		iam[iam_index] = i2c_smbus_read_byte_data(client, 0x00);
    		iam_index++;
//    		printk(KERN_CONT "%c", iam);
    	} while ( iam[iam_index-1] != 0x03);
    	iam[iam_index-1] = 0x00;
	dev_info(&client->dev,"%s", iam);
//        printk("%s", iam);
        printk("\n");
        
        
	return 0;
} 

MODULE_DEVICE_TABLE(i2c, edeye_id);

static struct i2c_driver edeye_driver = {
        .driver = {
                .name   = "ED-Eye",
                .of_match_table = of_match_ptr(edeye_of_match),
        },
        .probe_new      = edeye_probe,
        .id_table       = edeye_id,
};

module_i2c_driver(edeye_driver);

MODULE_AUTHOR("Maurizio Casti <maurizio.casti@iit.it>");
MODULE_DESCRIPTION("Event Driven Eye driver");
MODULE_LICENSE("GPL");
                        
