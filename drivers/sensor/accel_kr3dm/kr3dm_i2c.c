#include "kr3dm_i2c.h"

kr3dm_t *p_kr3dm;
kr3dm_t kr3dm;
kr3dmregs_t kr3dmregs;

static struct i2c_client *g_client;
static struct platform_device *kr3dm_accelerometer_device;
struct class *kr3dm_acc_class;

static unsigned short ignore[] = {I2C_CLIENT_END };
static unsigned short normal_addr[] = {I2C_CLIENT_END };
static unsigned short probe_addr[] = { 5, SENS_ADD >> 1, I2C_CLIENT_END };
#if defined(CONFIG_MACH_VINSQ)
#if 1//kimhyuns_vinsq_rev07
static unsigned short probe_addr_rev07[] = { 6, SENS_ADD >> 1, I2C_CLIENT_END };
#endif
#endif


static struct i2c_client_address_data addr_data = {
	.normal_i2c		= normal_addr,
	.probe			= probe_addr,
	.ignore			= ignore,
};

static struct platform_driver kr3dm_accelerometer_driver = {
	.probe 	 = kr3dm_accelerometer_probe,
	.suspend = kr3dm_accelerometer_suspend,
	.resume  = kr3dm_accelerometer_resume,
	.driver  = {
		.name = "kr3dm-accelerometer",
	}
};

struct file_operations kr3dm_acc_fops =
{
	.owner   = THIS_MODULE,
	.read    = kr3dm_read,
	.write   = kr3dm_write,
	.open    = kr3dm_open,
	.ioctl   = kr3dm_ioctl,
	.release = kr3dm_release,
};



struct i2c_driver acc_kr3dm_i2c_driver =
{
	.driver = {
		.name = "kr3dm_accelerometer_driver",
	},
	.attach_adapter	= &i2c_acc_kr3dm_attach_adapter,
	.detach_client	= &i2c_acc_kr3dm_detach_client,
};

char i2c_acc_kr3dm_read(u8 reg, u8 *val, unsigned int len )
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
char i2c_acc_kr3dm_write( u8 reg, u8 *val )
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
	printk("%s %d i2c transfer error\n", __func__, __LINE__);
#endif
	return err;
}



static int i2c_acc_kr3dm_detach_client(struct i2c_client *client)
{
	int err;

  	/* Try to detach the client from i2c space */
	if ((err = i2c_detach_client(client))) {
        return err;
	}
#ifdef DEBUG
	printk("[KR3DM] i2c_detach_client\n");
#endif
	kfree(client); /* Frees client data too, if allocated at the same time */
	g_client = NULL;
	return 0;
}



static int i2c_acc_kr3dm_attach_adapter(struct i2c_adapter *adapter)
{
#if defined(CONFIG_MACH_VINSQ)
#if 1// kimhyuns_vinsq_rev07
    if (system_rev>=CONFIG_INSTINCTQ_REV07)
        addr_data.probe=probe_addr_rev07;
#endif
#endif
	return i2c_probe(adapter, &addr_data, &i2c_acc_kr3dm_probe_client);

}


static int i2c_acc_kr3dm_probe_client(struct i2c_adapter *adapter, int address, int kind)
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
	new_client->driver = &acc_kr3dm_i2c_driver;
	new_client->flags = I2C_DF_NOTIFY | I2C_M_IGNORE_NAK;

	g_client = new_client;

	strlcpy(new_client->name, "kr3dm", I2C_NAME_SIZE);

	if ((err = i2c_attach_client(new_client)))
		goto ERROR1;
#ifdef DEBUG
	printk("i2c_acc_kr3dm_probe_client() completed!!!!!!!!!!!!!!!!!!\n");
#endif
	return 0;

	ERROR1:
		printk("i2c_acc_kr3dm_probe_client() ERROR1\n");
		kfree(new_client);
	ERROR0:
		printk("i2c_acc_kr3dm_probe_client() ERROR0\n");
    	return err;
}




int i2c_acc_kr3dm_init(void)
{
	int ret;

	if ( (ret = i2c_add_driver(&acc_kr3dm_i2c_driver)) )
	{
		printk("Driver registration failed, module not inserted.\n");
		return ret;
	}

	return 0;
}


void i2c_acc_kr3dm_exit(void)
{
	i2c_del_driver(&acc_kr3dm_i2c_driver);
}


int kr3dm_set_range(char range)
{
   int comres = 0;
   unsigned char data;

   if (p_kr3dm==0)
	    return E_KR3DM_NULL_PTR;

   if (range<3){
   		comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, CTRL_REG4, &data, 1 );
		data = data | (4 << range);
		comres += p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG4, &data, 1);
   }
   return comres;

}

int kr3dm_set_mode(unsigned char mode)
{

	int comres=0;
	unsigned char normal = 0x27;
	unsigned char sleep = 0x00;

	if (p_kr3dm==0)
		return E_KR3DM_NULL_PTR;

	switch(mode)
	{
		case KR3DM_MODE_NORMAL:
		case KR3DM_MODE_WAKE_UP:
			comres += p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG1, &normal, 1);
			break;
		case KR3DM_MODE_SLEEP:
			comres += p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG1, &sleep, 1);
			break;
		default:
			return E_OUT_OF_RANGE;
	}
	p_kr3dm->mode = mode;

	return comres;

}

unsigned char kr3dm_get_mode(void)
{
    if (p_kr3dm==0)
    	return E_KR3DM_NULL_PTR;

	return p_kr3dm->mode;

}

int kr3dm_set_bandwidth(char bw)
{
	int comres = 0;
	unsigned char data = 0x27;

	if (p_kr3dm==0)
		return E_KR3DM_NULL_PTR;

	if (bw<8)
	{
	  data = data | (3 << bw);
	  comres += p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG1, &data, 1 );
	}

	return comres;
}


int kr3dm_get_bandwidth(unsigned char *bw)
{
	int comres = 1;

	if (p_kr3dm==0)
		return E_KR3DM_NULL_PTR;

	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, CTRL_REG1, bw, 1 );

	*bw = (*bw & 0x18);

	return comres;
}

int kr3dm_open (struct inode *inode, struct file *filp)
{
	return 0;
}

ssize_t kr3dm_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

ssize_t kr3dm_write (struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}

int kr3dm_release (struct inode *inode, struct file *filp)
{
	return 0;
}

int kr3dm_ioctl(struct inode *inode, struct file *filp, unsigned int cmd,  unsigned long arg)
{

	int err = 0;
	unsigned char data[3];
	kr3dmacc_t accels;
	unsigned char val1 = 0x27;

	/* check cmd */
	if(_IOC_TYPE(cmd) != KR3DM_IOC_MAGIC)
	{
		printk("cmd magic type error\n");
		return -ENOTTY;
	}
	if(_IOC_NR(cmd) > KR3DM_IOC_MAXNR)
	{
		printk("cmd number error\n");
		return -ENOTTY;
	}

	if(_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,(void __user*)arg, _IOC_SIZE(cmd));
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	if(err)
	{
		printk("cmd access_ok error\n");
		return -EFAULT;
	}

	switch(cmd)
	{
		case KR3DM_READ_ACCEL_XYZ:
			p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG1, &val1, 1 );
			err = kr3dm_read_accel_xyz(&accels);
			if(copy_to_user((kr3dmacc_t*)arg, &accels, sizeof(kr3dmacc_t))!=0)
			{
				printk("copy_to error\n");
				return -EFAULT;
			}
			return err;

		case KR3DM_SET_RANGE:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
				printk("[KR3DM] copy_from_user error\n");
				return -EFAULT;
			}
			err = kr3dm_set_range(*data);
			return err;

		case KR3DM_SET_MODE:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
				printk("[KR3DM] copy_from_user error\n");
				return -EFAULT;
			}
			err = kr3dm_set_mode(*data);
			return err;

		case KR3DM_SET_BANDWIDTH:
			if(copy_from_user(data,(unsigned char*)arg,1)!=0)
			{
				printk("[KR3DM] copy_from_user error\n");
				return -EFAULT;
			}
			err = kr3dm_set_bandwidth(*data);
			return err;

		default:
			return 0;
	}
	return 0;
}


int kr3dm_read_accel_xyz(kr3dmacc_t * acc)
{
	int comres;
	unsigned char data[3];

	if (p_kr3dm==0)
		return E_KR3DM_NULL_PTR;

	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_X_L, &data[0], 1);
	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_Y_L, &data[1], 1);
	comres = p_kr3dm->KR3DM_BUS_READ_FUNC(p_kr3dm->dev_addr, OUT_Z_L, &data[2], 1);

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
static void kr3dm_early_suspend(struct early_suspend *handler)
{

	kr3dm_set_mode( KR3DM_MODE_SLEEP );
}

static void kr3dm_late_resume(struct early_suspend *handler)
{

	kr3dm_set_mode( KR3DM_MODE_NORMAL );
}
#endif /* CONFIG_HAS_EARLYSUSPEND */


void kr3dm_chip_init(void)
{

	kr3dm.kr3dm_bus_write = i2c_acc_kr3dm_write;
	kr3dm.kr3dm_bus_read  = i2c_acc_kr3dm_read;

#ifdef CONFIG_HAS_EARLYSUSPEND
	kr3dm.early_suspend.suspend = kr3dm_early_suspend;
	kr3dm.early_suspend.resume = kr3dm_late_resume;
	register_early_suspend(&kr3dm.early_suspend);
#endif
	kr3dm_init( &kr3dm );
}


int kr3dm_init(kr3dm_t *kr3dm)
{

	unsigned char val1 = 0x27;
	unsigned char val2 = 0x00;

	p_kr3dm = kr3dm;
	p_kr3dm->dev_addr = SENS_ADD;										/* preset KR3DM I2C_addr */
	p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG1, &val1, 1 );
	p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG2, &val2, 1 );
	p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG3, &val2, 1 );
	p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG4, &val2, 1 );
	p_kr3dm->KR3DM_BUS_WRITE_FUNC(p_kr3dm->dev_addr, CTRL_REG5, &val2, 1 );
;

	return 0;
}

int kr3dm_acc_start(void)
{
	int result;
	struct device *dev_t;

	kr3dmacc_t accels; /* only for test */

	result = register_chrdev( KR3DM_MAJOR, "kr3dm", &kr3dm_acc_fops);

	if (result < 0)
	{
		return result;
	}

	kr3dm_acc_class = class_create (THIS_MODULE, "KR3DM-dev");

	if (IS_ERR(kr3dm_acc_class))
	{
		unregister_chrdev( KR3DM_MAJOR, "kr3dm" );
		return PTR_ERR( kr3dm_acc_class );
	}

	dev_t = device_create( kr3dm_acc_class, NULL, MKDEV(KR3DM_MAJOR, 0), "%s", "kr3dm");

	if (IS_ERR(dev_t))
	{
		return PTR_ERR(dev_t);
	}

	result = i2c_acc_kr3dm_init();

	if(result)
	{
		return result;
	}

	kr3dm_chip_init();
#ifdef DEBUG
	printk("[KR3DM] read_xyz ==========================\n");
	kr3dm_read_accel_xyz( &accels );
	printk("[KR3DM] x = %d  /  y =  %d  /  z = %d\n", accels.x, accels.y, accels.z );
	printk("[KR3DM] ======================kr3dm_acc_start Ready for use !!!!! =============\n");
#endif
	return 0;
}



static int kr3dm_accelerometer_suspend( struct platform_device* pdev, pm_message_t state )
{
	return 0;
}


static int kr3dm_accelerometer_resume( struct platform_device* pdev )
{
	return 0;
}

void kr3dm_acc_end(void)
{
	unregister_chrdev( KR3DM_MAJOR, "krx3dm" );

	i2c_acc_kr3dm_exit();

	device_destroy( kr3dm_acc_class, MKDEV(KR3DM_MAJOR, 0) );
	class_destroy( kr3dm_acc_class );
	unregister_early_suspend(&kr3dm.early_suspend);
}

static int kr3dm_accelerometer_probe( struct platform_device* pdev )
{
	return kr3dm_acc_start();

}


static int __init kr3dm_acc_init(void)
{
	int result;

	result = platform_driver_register( &kr3dm_accelerometer_driver);
#ifdef DEBUG
	printk("[KR3DM] ********** kr3dm_acc_init =====================\n");
#endif
	if( result )
	{
		return result;
	}

	kr3dm_accelerometer_device  = platform_device_register_simple( "kr3dm-accelerometer", -1, NULL, 0 );

	if( IS_ERR( kr3dm_accelerometer_device ) )
	{
		return PTR_ERR( kr3dm_accelerometer_device );
	}

	return 0;
}


static void __exit kr3dm_acc_exit(void)
{
	kr3dm_acc_end();
	platform_device_unregister( kr3dm_accelerometer_device );
	platform_driver_unregister( &kr3dm_accelerometer_driver );
}


module_init( kr3dm_acc_init );
module_exit( kr3dm_acc_exit );

MODULE_AUTHOR("vishnu.p");
MODULE_DESCRIPTION("accelerometer driver for KR3DM");
MODULE_LICENSE("GPL");
