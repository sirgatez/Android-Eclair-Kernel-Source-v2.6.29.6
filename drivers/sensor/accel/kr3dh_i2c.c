#include "kr3dh_i2c.h"

kr3dh_t *p_kr3dh;
kr3dh_t kr3dh;
kr3dhregs_t kr3dhregs;

static struct i2c_client *g_client;
static struct platform_device *kr3dh_accelerometer_device;
struct class *kr3dh_acc_class;

static unsigned short ignore[] = {I2C_CLIENT_END };
static unsigned short normal_addr[] = {I2C_CLIENT_END };
static unsigned short probe_addr[] = { 5, SENS_ADD >> 1, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c		= normal_addr,
	.probe			= probe_addr,
	.ignore			= ignore,
};

static struct platform_driver kr3dh_accelerometer_driver = {
	.probe 	 = kr3dh_accelerometer_probe,
	.suspend = kr3dh_accelerometer_suspend,
	.resume  = kr3dh_accelerometer_resume,
	.driver  = {
		.name = "kr3dh-accelerometer",
	}
};

struct file_operations kr3dh_acc_fops =
{
	.owner   = THIS_MODULE,
};

struct i2c_driver acc_kr3dh_i2c_driver =
{
	.driver = {
		.name = "kr3dh_accelerometer_driver",
	},
	.attach_adapter	= &i2c_acc_kr3dh_attach_adapter,
	.detach_client	= &i2c_acc_kr3dh_detach_client,
};

static ssize_t show_xyz(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	kr3dhacc_t accels;
	kr3dh_read_accel_xyz(&accels);
	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", accels.x, accels.y,
			accels.z);
}

static ssize_t store_xyz(struct device *dev, const char *buf)
{
	printk("doing nothing\n");
	return strnlen(buf, PAGE_SIZE);
}

static DEVICE_ATTR(xyz, S_IWUSR | S_IRUGO, show_xyz, store_xyz);

char i2c_acc_kr3dh_read(u8 reg, u8 *val, unsigned int len )
{
	int 	 err;
	struct 	 i2c_msg msg[1];
	unsigned char data[1];

	if( (g_client == NULL) || (!g_client->adapter) )
	{
		return -ENODEV;
	}

	msg->addr 	= g_client->addr;
	msg->flags 	= I2C_M_WR;
	msg->len 	= 1;
	msg->buf 	= data;
	*data       = reg;

	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0)
	{
		msg->flags = I2C_M_RD;
		msg->len   = len;
		msg->buf   = val;
		err = i2c_transfer(g_client->adapter, msg, 1);
	}

	if (err >= 0)
	{
		return 0;
	}
#ifdef DEBUG
	printk("%s %d i2c transfer error\n", __func__, __LINE__);
#endif
	return err;;

}

char i2c_acc_kr3dh_write( u8 reg, u8 *val )
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if( (g_client == NULL) || (!g_client->adapter) ){
		return -ENODEV;
	}

	data[0] = reg;
	data[1] = *val;

	msg->addr = g_client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;

	err = i2c_transfer(g_client->adapter, msg, 1);

	if (err >= 0) return 0;
#ifdef DEBUG
	printk(KERN_ERR "%s %d i2c transfer error\n", __func__, __LINE__);
#endif
	return err;
}



static int i2c_acc_kr3dh_detach_client(struct i2c_client *client)
{
	int err;

  	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
        return err;
	}
#ifdef DEBUG
	printk("[KR3DH] i2c_detach_client\n");
#endif
	kfree(client); /* Frees client data too, if allocated at the same time */
	g_client = NULL;
	return 0;
}

static int i2c_acc_kr3dh_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, &i2c_acc_kr3dh_probe_client);
}

static int i2c_acc_kr3dh_probe_client(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	int err = 0;


	if ( !i2c_check_functionality(adapter,I2C_FUNC_SMBUS_BYTE_DATA) ) {
		printk(KERN_INFO "byte op is not permited.\n");
		goto ERROR0;
	}

	new_client = kzalloc(sizeof(struct i2c_client), GFP_KERNEL );

	if ( !new_client )  {
		err = -ENOMEM;
		goto ERROR0;
	}

	new_client->addr = address;
 	new_client->adapter = adapter;
	new_client->driver = &acc_kr3dh_i2c_driver;
	new_client->flags = I2C_DF_NOTIFY | I2C_M_IGNORE_NAK;

	g_client = new_client;

	strlcpy(new_client->name, "kr3dh", I2C_NAME_SIZE);

	if ((err = i2c_attach_client(new_client)))
		goto ERROR1;
#ifdef DEBUG
	printk("i2c_acc_kr3dh_probe_client() completed!!!!!!!!!!!!!!!!!!\n");
#endif
	return 0;

	ERROR1:
		printk(KERN_ERR "i2c_acc_kr3dh_probe_client() ERROR1\n");
		kfree(new_client);
	ERROR0:
		printk(KERN_ERR "i2c_acc_kr3dh_probe_client() ERROR0\n");
    	return err;
}




int i2c_acc_kr3dh_init(void)
{
	int ret;

	if ( (ret = i2c_add_driver(&acc_kr3dh_i2c_driver)) )
	{
		printk(KERN_ERR "Driver registration failed, module not inserted.\n");
		return ret;
	}

	return 0;
}


void i2c_acc_kr3dh_exit(void)
{
	i2c_del_driver(&acc_kr3dh_i2c_driver);
}


int kr3dh_set_range(char range)
{
   int comres = 0;
   unsigned char data;

   if (p_kr3dh==0)
	    return E_KR3DH_NULL_PTR;

   if (range<3){
   		comres = p_kr3dh->KR3DH_BUS_READ_FUNC(p_kr3dh->dev_addr, CTRL_REG4, &data, 1 );
		data = data | (4 << range);
		comres += p_kr3dh->KR3DH_BUS_WRITE_FUNC(p_kr3dh->dev_addr, CTRL_REG4, &data, 1);
   }
   return comres;

}

int kr3dh_set_mode(unsigned char mode)
{

	int comres=0;
	unsigned char normal = 0x27;
	unsigned char sleep = 0x00;

	if (p_kr3dh==0)
		return E_KR3DH_NULL_PTR;

	switch(mode)
	{
		case KR3DH_MODE_NORMAL:
		case KR3DH_MODE_WAKE_UP:
			comres += p_kr3dh->KR3DH_BUS_WRITE_FUNC(p_kr3dh->dev_addr, CTRL_REG1, &normal, 1);
			break;
		case KR3DH_MODE_SLEEP:
			comres += p_kr3dh->KR3DH_BUS_WRITE_FUNC(p_kr3dh->dev_addr, CTRL_REG1, &sleep, 1);
			break;
		default:
			return E_OUT_OF_RANGE;
	}
	p_kr3dh->mode = mode;

	return comres;
}

unsigned char kr3dh_get_mode(void)
{
    if (p_kr3dh==0)
    	return E_KR3DH_NULL_PTR;

	return p_kr3dh->mode;

}

int kr3dh_set_bandwidth(char bw)
{
	int comres = 0;
	unsigned char data = 0x27;

	if (p_kr3dh==0)
		return E_KR3DH_NULL_PTR;

	if (bw<8)
	{
	  data = data | (3 << bw);
	  comres += p_kr3dh->KR3DH_BUS_WRITE_FUNC(p_kr3dh->dev_addr, CTRL_REG1, &data, 1 );
	}

	return comres;
}


int kr3dh_get_bandwidth(unsigned char *bw)
{
	int comres = 1;

	if (p_kr3dh==0)
		return E_KR3DH_NULL_PTR;

	comres = p_kr3dh->KR3DH_BUS_READ_FUNC(p_kr3dh->dev_addr, CTRL_REG1, bw, 1 );

	*bw = (*bw & 0x18);

	return comres;
}


int kr3dh_read_accel_xyz(kr3dhacc_t * acc)
{
	int comres;
	unsigned char data[3];

	if (p_kr3dh==0)
		return E_KR3DH_NULL_PTR;

	comres = p_kr3dh->KR3DH_BUS_READ_FUNC(p_kr3dh->dev_addr, OUT_X_L, &data[0], 1);
	comres = p_kr3dh->KR3DH_BUS_READ_FUNC(p_kr3dh->dev_addr, OUT_Y_L, &data[1], 1);
	comres = p_kr3dh->KR3DH_BUS_READ_FUNC(p_kr3dh->dev_addr, OUT_Z_L, &data[2], 1);

	data[0] = (~data[0] + 1);
	data[1] = (~data[1] + 1);
	data[2] = (~data[2] + 1);

	if(1) // system_rev>=CONFIG_INSTINCTQ_REV05)
	{
		if(data[0] & 0x80)
			acc->x = (0x100-data[0])*(-1);
		else
			acc->x = ((data[0]) & 0xFF);
		if(data[1]& 0x80)
			acc->y = (0x100-data[1])*(-1);
		else
			acc->y = ((data[1]) & 0xFF);
		if(data[2]& 0x80)
			acc->z = (0x100-data[2])*(-1);
		else
			acc->z = ((data[2]) & 0xFF);
	}
	else
	{
		if(data[0] & 0x80)
			acc->y = -(0x100-data[0])*(-1);
		else
			acc->y = -((data[0]) & 0xFF);
		if(data[1]& 0x80)
			acc->x = (0x100-data[1])*(-1);
		else
			acc->x = ((data[1]) & 0xFF);
		if(data[2]& 0x80)
			acc->z = (0x100-data[2])*(-1);
		else
			acc->z = ((data[2]) & 0xFF);
	}

	return comres;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kr3dh_early_suspend(struct early_suspend *handler)
{
    printk("%s \n",__FUNCTION__);
	kr3dh_set_mode( KR3DH_MODE_SLEEP );
}

static void kr3dh_late_resume(struct early_suspend *handler)
{
    printk("%s \n",__FUNCTION__);
	kr3dh_set_mode( KR3DH_MODE_NORMAL );
}
#endif /* CONFIG_HAS_EARLYSUSPEND */


void kr3dh_chip_init(void)
{

	kr3dh.kr3dh_bus_write = i2c_acc_kr3dh_write;
	kr3dh.kr3dh_bus_read  = i2c_acc_kr3dh_read;

#ifdef CONFIG_HAS_EARLYSUSPEND
	kr3dh.early_suspend.suspend = kr3dh_early_suspend;
	kr3dh.early_suspend.resume = kr3dh_late_resume;
	register_early_suspend(&kr3dh.early_suspend);
#endif
	kr3dh_init( &kr3dh );
}


int kr3dh_init(kr3dh_t *kr3dh)
{

	unsigned char val1 = 0x27;
	unsigned char val2 = 0x00;

	p_kr3dh = kr3dh;
	p_kr3dh->dev_addr = SENS_ADD;										/* preset KR3DH I2C_addr */
	p_kr3dh->KR3DH_BUS_WRITE_FUNC(p_kr3dh->dev_addr, CTRL_REG1, &val1, 1 );
	p_kr3dh->KR3DH_BUS_WRITE_FUNC(p_kr3dh->dev_addr, CTRL_REG2, &val2, 1 );
	p_kr3dh->KR3DH_BUS_WRITE_FUNC(p_kr3dh->dev_addr, CTRL_REG3, &val2, 1 );
	p_kr3dh->KR3DH_BUS_WRITE_FUNC(p_kr3dh->dev_addr, CTRL_REG4, &val2, 1 );
	p_kr3dh->KR3DH_BUS_WRITE_FUNC(p_kr3dh->dev_addr, CTRL_REG5, &val2, 1 );

	return 0;
}

int kr3dh_acc_start(void)
{
	int result;
	struct device *dev_t;

	kr3dhacc_t accels; /* only for test */

	result = register_chrdev( KR3DH_MAJOR, "kr3dh", &kr3dh_acc_fops);

	if (result < 0)
	{
		return result;
	}

	kr3dh_acc_class = class_create (THIS_MODULE, "KR3DH-dev");

	if (IS_ERR(kr3dh_acc_class))
	{
		unregister_chrdev( KR3DH_MAJOR, "kr3dh" );
		return PTR_ERR( kr3dh_acc_class );
	}

	dev_t = device_create( kr3dh_acc_class, NULL, MKDEV(KR3DH_MAJOR, 0), "%s", "kr3dh");

	if (IS_ERR(dev_t))
	{
		return PTR_ERR(dev_t);
	}

	if (device_create_file(dev_t, &dev_attr_xyz) < 0)
		printk(KERN_ERR "Failed to create device file(%s)!\n", dev_attr_xyz.attr.name);

	result = i2c_acc_kr3dh_init();

	if(result)
	{
		return result;
	}

	kr3dh_chip_init();
//#ifdef DEBUG
	printk("[KR3DH] read_xyz ==========================\n");
	kr3dh_read_accel_xyz( &accels );
	printk("[KR3DH] x = %d  /  y =  %d  /  z = %d\n", accels.x, accels.y, accels.z );
	printk("[KR3DH] ======================kr3dh_acc_start Ready for use !!!!! =============\n");
//#endif
	return 0;
}



static int kr3dh_accelerometer_suspend( struct platform_device* pdev, pm_message_t state )
{
	return 0;
}


static int kr3dh_accelerometer_resume( struct platform_device* pdev )
{
	return 0;
}

void kr3dh_acc_end(void)
{
	unregister_chrdev( KR3DH_MAJOR, "kr3dh" );

	i2c_acc_kr3dh_exit();

	device_destroy( kr3dh_acc_class, MKDEV(KR3DH_MAJOR, 0) );
	class_destroy( kr3dh_acc_class );
	unregister_early_suspend(&kr3dh.early_suspend);
}

static int kr3dh_accelerometer_probe( struct platform_device* pdev )
{
	return kr3dh_acc_start();
}


static int __init kr3dh_acc_init(void)
{
	int result;

//#ifdef DEBUG
	printk("[KR3DH] ********** kr3dh_acc_init =====================\n");
//#endif

	result = platform_driver_register( &kr3dh_accelerometer_driver);
	if( result )
	{
		return result;
	}

	kr3dh_accelerometer_device  = platform_device_register_simple( "kr3dh-accelerometer", -1, NULL, 0 );

	if( IS_ERR( kr3dh_accelerometer_device ) )
	{
		return PTR_ERR( kr3dh_accelerometer_device );
	}

	return 0;
}


static void __exit kr3dh_acc_exit(void)
{
	kr3dh_acc_end();
	platform_device_unregister( kr3dh_accelerometer_device );
	platform_driver_unregister( &kr3dh_accelerometer_driver );
}


module_init( kr3dh_acc_init );
module_exit( kr3dh_acc_exit );

MODULE_AUTHOR("vishnu.p");
MODULE_DESCRIPTION("accelerometer driver for KR3DH");
MODULE_LICENSE("GPL");
