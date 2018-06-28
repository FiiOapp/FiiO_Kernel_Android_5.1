/* drivers/i2c/chips/mma8653.c - mma8653 compass driver
 *
 * Copyright (C) 2007-2008 HTC Corporation.
 * Author: Hou-Kun Chen <houkun.chen@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/mma8653.h>
#include <mach/gpio.h>
#include <mach/board.h> 

#if 0
#define mmaprintk(x...) printk(x)
#else
#define mmaprintk(x...)
#endif

#if 0
#define mmaprintkd(x...) printk(x)
#else
#define mmaprintkd(x...)
#endif

#if 0
#define mmaprintkf(x...) printk(x)
#else
#define mmaprintkf(x...)
#endif

//reg2
#define SLPE           (1 << 2)
#define SELFTEST       (1 << 7)
#define SOFT_RESET     (1 << 6)

//reg3
#define WAKE_LNDPRT    (1 << 5)
#define WAKE_FF_MT     (1 << 3)
#define IPOL           (1 << 1)
#define PP_OD          (1 << 0)

//reg4
#define INT_EN_ASLP    (1 << 7)
#define INT_EN_LNDPRT  (1 << 4)
#define INT_EN_FF_MT   (1 << 2)
#define INT_EN_DRDY    (1 << 0)

//reg5
#define INT_CFG_ASLP   (1 << 7)
#define INT_CFG_LNDPRT (1 << 4)
#define INT_CFG_FF_MT  (1 << 2)
#define INT_CFG_DRDY   (1 << 0)

static int  mma8653_probe(struct i2c_client *client, const struct i2c_device_id *id);

#define MMA8653_SPEED		200 * 1000
#define MMA8451_DEVID		0x1a
#define MMA8653_DEVID		0x2a
#define MMA8453_DEVID		0x5a//0x3a
/* Addresses to scan -- protected by sense_data_mutex */
//static char sense_data[RBUFF_SIZE + 1];
static struct i2c_client *this_client;
static struct miscdevice mma8653_device;

static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);

static int revision = -1;
static const char* vendor = "Freescale Semiconductor";
static char devid;


typedef char status_t;
/*status*/
#define MMA8653_OPEN           1
#define MMA8653_CLOSE          0

struct mma8653_data {
    status_t status;
    char  curr_tate;
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct work_struct work;
	struct delayed_work delaywork;	/*report second event*/
    
    struct mma8653_axis sense_data;
    struct mutex sense_data_mutex;
    atomic_t data_ready;
    wait_queue_head_t data_ready_wq;

    int start_count;
    struct mutex operation_mutex;
};

/* AKM HW info */
static ssize_t gsensor_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	// sprintf(buf, "%#x\n", revision);
	sprintf(buf, "%s.\n", vendor);
	ret = strlen(buf) + 1;

	return ret;
}

static DEVICE_ATTR(vendor, 0444, gsensor_vendor_show, NULL);

static struct kobject *android_gsensor_kobj;

static int gsensor_sysfs_init(void)
{
	int ret ;

	android_gsensor_kobj = kobject_create_and_add("android_gsensor", NULL);
	if (android_gsensor_kobj == NULL) {
		mmaprintk(KERN_ERR
		       "MMA8653 gsensor_sysfs_init:"\
		       "subsystem_register failed\n");
		ret = -ENOMEM;
		goto err;
	}

	ret = sysfs_create_file(android_gsensor_kobj, &dev_attr_vendor.attr);   // "vendor"
	if (ret) {
		mmaprintk(KERN_ERR
		       "MMA8653 gsensor_sysfs_init:"\
		       "sysfs_create_group failed\n");
		goto err4;
	}

	return 0 ;
err4:
	kobject_del(android_gsensor_kobj);
err:
	return ret ;
}

static int mma8653_rx_data(struct i2c_client *client, char *rxData, int length)
{
	int ret = 0;
	char reg = rxData[0];
	ret = i2c_master_reg8_recv(client, reg, rxData, length, MMA8653_SPEED);
	return (ret > 0)? 0 : ret;
}

static int mma8653_tx_data(struct i2c_client *client, char *txData, int length)
{
	int ret = 0;
	char reg = txData[0];
	ret = i2c_master_reg8_send(client, reg, &txData[1], length-1, MMA8653_SPEED);
	return (ret > 0)? 0 : ret;
}

static char mma845x_read_reg(struct i2c_client *client,int addr)
{
	char tmp;
	int ret = 0;

	tmp = addr;
//	ret = mma8653_tx_data(client, &tmp, 1);
	ret = mma8653_rx_data(client, &tmp, 1);
	return tmp;
}

static int mma845x_write_reg(struct i2c_client *client,int addr,int value)
{
	char buffer[3];
	int ret = 0;

	buffer[0] = addr;
	buffer[1] = value;
	ret = mma8653_tx_data(client, &buffer[0], 2);
	return ret;
}


static char mma8653_get_devid(struct i2c_client *client)
{
	printk("mma8653 devid:%x\n",mma845x_read_reg(client,MMA8653_REG_WHO_AM_I));
	return mma845x_read_reg(client,MMA8653_REG_WHO_AM_I);
}

static int mma845x_active(struct i2c_client *client,int enable)
{
	int tmp;
	int ret = 0;
	
	tmp = mma845x_read_reg(client,MMA8653_REG_CTRL_REG1);
	if(enable)
		tmp |=ACTIVE_MASK;
	else
		tmp &=~ACTIVE_MASK;
	mmaprintkd("mma845x_active %s (0x%x)\n",enable?"active":"standby",tmp);	
	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG1,tmp);
	return ret;
}

static int mma8653_start_test(struct i2c_client *client)
{
	int ret = 0;
	int tmp;

	mmaprintkf("-------------------------mma8653 start test------------------------\n");	
	
	/* standby */
	mma845x_active(client,0);
	mmaprintkd("mma8653 MMA8653_REG_SYSMOD:%x\n",mma845x_read_reg(client,MMA8653_REG_SYSMOD));

	/* disable FIFO  FMODE = 0*/
	ret = mma845x_write_reg(client,MMA8653_REG_F_SETUP,0);
	mmaprintkd("mma8653 MMA8653_REG_F_SETUP:%x\n",mma845x_read_reg(client,MMA8653_REG_F_SETUP));

	/* set full scale range to 2g */
	ret = mma845x_write_reg(client,MMA8653_REG_XYZ_DATA_CFG,0);
	mmaprintkd("mma8653 MMA8653_REG_XYZ_DATA_CFG:%x\n",mma845x_read_reg(client,MMA8653_REG_XYZ_DATA_CFG));

	/* set bus 8bit/14bit(FREAD = 1,FMODE = 0) ,data rate*/
	tmp = (MMA8653_RATE_12P5<< MMA8653_RATE_SHIFT) | FREAD_MASK;
	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG1,tmp);
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG1:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG1));
	
	mmaprintkd("mma8653 MMA8653_REG_SYSMOD:%x\n",mma845x_read_reg(client,MMA8653_REG_SYSMOD));

	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG3,5);
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG3:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG3));
	
	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG4,1);
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG4:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG4));

	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG5,1);
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG5:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG5));	

	mmaprintkd("mma8653 MMA8653_REG_SYSMOD:%x\n",mma845x_read_reg(client,MMA8653_REG_SYSMOD));
	mma845x_active(client,1);
	mmaprintkd("mma8653 MMA8653_REG_SYSMOD:%x\n",mma845x_read_reg(client,MMA8653_REG_SYSMOD));

	enable_irq(client->irq);
	msleep(50);

	return ret;
}

static int mma8653_init_reg(struct i2c_client *client, char rate)
{
        int ret = 0;
        int byte;

        /* range mode */
        byte = mma845x_read_reg(client, MMA8653_REG_XYZ_DATA_CFG);
        byte &= ~0x03;
        byte |= 0x0;
        mma845x_write_reg(client, MMA8653_REG_XYZ_DATA_CFG, byte); 

       /* data rates */
       byte = 0;
       byte = mma845x_read_reg(client, MMA8653_REG_CTRL_REG1);
       byte &= ~(0x38);
       byte |= 0x20;
       byte &= ~(0xc0);
       byte |= 0x80;
       ret = mma845x_write_reg(client, MMA8653_REG_CTRL_REG1, byte);

       mmaprintkd("mma8653 MMA8653_REG_SYSMOD:%x\n",mma845x_read_reg(client,MMA8653_REG_SYSMOD));

	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG3, 5);
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG3:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG3));
	
	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG4, (INT_EN_ASLP | INT_EN_LNDPRT | INT_EN_FF_MT | INT_EN_DRDY));
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG4:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG4));

	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG5, (INT_CFG_ASLP | INT_CFG_LNDPRT | INT_CFG_FF_MT | INT_CFG_DRDY));
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG5:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG5));

ret = mma845x_write_reg(client,MMA8653_REG_OFF_X, 250);
	mmaprintkd("mma8653 MMA8653_REG_OFF_X:%x\n",mma845x_read_reg(client,MMA8653_REG_OFF_X));

ret = mma845x_write_reg(client,MMA8653_REG_OFF_Y, 250);
	mmaprintkd("mma8653 MMA8653_REG_OFF_Y:%x\n",mma845x_read_reg(client,MMA8653_REG_OFF_Y));
ret = mma845x_write_reg(client,MMA8653_REG_OFF_Z, 250);
	mmaprintkd("mma8653 MMA8653_REG_OFF_Z:%x\n",mma845x_read_reg(client,MMA8653_REG_OFF_Z));

       return ret;
}

static int mma8653_start_dev(struct i2c_client *client, char rate)
{
	int ret = 0;
	int tmp;
	struct mma8653_data *mma8653 = (struct mma8653_data *)i2c_get_clientdata(client);   // mma8653_data \B6\A8\D2\E5\D4\DA mma8653.h \D6\D0. 

	mmaprintkf("-------------------------mma8653 start ------------------------\n");	


	/* standby */
	mma845x_active(client,0);
	mmaprintkd("mma8653 MMA8653_REG_SYSMOD:%x\n",mma845x_read_reg(client,MMA8653_REG_SYSMOD));

//mmaprintkd("mma8653 MMA8653_REG_CTRL_REG2:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG2));
//ret = mma845x_write_reg(client, MMA8653_REG_CTRL_REG2, 0x40);
//mmaprintkd("mma8653 MMA8653_REG_CTRL_REG2:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG2));

//printk("mma8653_init_reg begin!!!\n\n");
//        ret = mma8653_init_reg(client, rate);
//printk("mma8653_init_reg end!!!!!\n\n");
#if 1
	/* disable FIFO  FMODE = 0*/
	ret = mma845x_write_reg(client,MMA8653_REG_F_SETUP,0);
	mmaprintkd("mma8653 MMA8653_REG_F_SETUP:%x\n",mma845x_read_reg(client,MMA8653_REG_F_SETUP));

	/* set full scale range to 2g */
	ret = mma845x_write_reg(client,MMA8653_REG_XYZ_DATA_CFG,0);
	mmaprintkd("mma8653 MMA8653_REG_XYZ_DATA_CFG:%x\n",mma845x_read_reg(client,MMA8653_REG_XYZ_DATA_CFG));

	/* set bus 8bit/14bit(FREAD = 1,FMODE = 0) ,data rate*/
	tmp = (rate<< MMA8653_RATE_SHIFT) | FREAD_MASK;
	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG1,tmp);
	mma8653->curr_tate = rate;
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG1:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG1));
	
	mmaprintkd("mma8653 MMA8653_REG_SYSMOD:%x\n",mma845x_read_reg(client,MMA8653_REG_SYSMOD));

	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG3,5);
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG3:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG3));
	
	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG4,1);
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG4:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG4));

	ret = mma845x_write_reg(client,MMA8653_REG_CTRL_REG5,1);
	mmaprintkd("mma8653 MMA8653_REG_CTRL_REG5:%x\n",mma845x_read_reg(client,MMA8653_REG_CTRL_REG5));	
#endif

	mmaprintkd("mma8653 MMA8653_REG_SYSMOD:%x\n",mma845x_read_reg(client,MMA8653_REG_SYSMOD));
	mma845x_active(client,1);
	mmaprintkd("mma8653 MMA8653_REG_SYSMOD:%x\n",mma845x_read_reg(client,MMA8653_REG_SYSMOD));
	
	enable_irq(client->irq);
	return ret;

}

static int mma8653_start(struct i2c_client *client, char rate)
{ 
    struct mma8653_data *mma8653 = (struct mma8653_data *)i2c_get_clientdata(client);
    
   mmaprintkf("%s::enter\n",__FUNCTION__); 
    if (mma8653->status == MMA8653_OPEN) {
        return 0;      
    }
    mma8653->status = MMA8653_OPEN;
    return mma8653_start_dev(client, rate);
}

static int mma8653_close_dev(struct i2c_client *client)
{    	
	disable_irq_nosync(client->irq);
	return mma845x_active(client,0);
}

static int mma8653_close(struct i2c_client *client)
{
    struct mma8653_data *mma8653 = (struct mma8653_data *)i2c_get_clientdata(client);
   mmaprintkf("%s::enter\n",__FUNCTION__); 
    mma8653->status = MMA8653_CLOSE;
    
    return mma8653_close_dev(client);
}

static int mma8653_reset_rate(struct i2c_client *client, char rate)
{
	int ret = 0;
	
	mmaprintkf("\n----------------------------mma8653_reset_rate------------------------\n");
	
    ret = mma8653_close_dev(client);
    ret = mma8653_start_dev(client, rate);
  
	return ret ;
}

static inline int mma8653_convert_to_int(const char high_byte, const char low_byte)
{
    s64 result;
/* enabled only if FREAD MODE */
/*
    if (high_byte < MMA8653_BOUNDARY) {
       result = high_byte * MMA8653_GRAVITY_STEP;
    } else {
       result = ~(((~high_byte&0x7f) + 1) * MMA8653_GRAVITY_STEP) + 1;
    }
*/
	switch (devid) {

		case MMA8451_DEVID:
			result = ((int)high_byte << (MMA8451_PRECISION-8)) 
					| ((int)low_byte >> (16-MMA8451_PRECISION));
			if (result < MMA8451_BOUNDARY)
       			result = result* MMA8451_GRAVITY_STEP;
    		else
       			result = ~( ((~result & (0x7fff>>(16-MMA8451_PRECISION)) ) + 1) 
			   			* MMA8451_GRAVITY_STEP) + 1;
			break;

		case MMA8653_DEVID:
			result = ((int)high_byte << (MMA8653_PRECISION-8)) 
					| ((int)low_byte >> (16-MMA8653_PRECISION));
			if (result < MMA8653_BOUNDARY)
       			result = result* MMA8653_GRAVITY_STEP;
    		else
       			result = ~( ((~result & (0x7fff>>(16-MMA8653_PRECISION)) ) + 1) 
			   			* MMA8653_GRAVITY_STEP) + 1;
			break;
			
		case MMA8453_DEVID:
			result = ((int)high_byte << (MMA8453_PRECISION-8)) 
					| ((int)low_byte >> (16-MMA8453_PRECISION));
			if (result < MMA8453_BOUNDARY)
       			result = result* MMA8453_GRAVITY_STEP;
    		else
       			result = ~( ((~result & (0x7fff>>(16-MMA8453_PRECISION)) ) + 1) 
			   			* MMA8453_GRAVITY_STEP) + 1;
			break;

		default:
			mmaprintk(KERN_ERR
		       "mma8653_convert_to_int: devid wasn't set correctly\n");
			return -EFAULT;
    }

    return (int)result;
}

static void mma8653_report_value(struct i2c_client *client, struct mma8653_axis *axis)
{
	struct mma8653_data *mma8653 = i2c_get_clientdata(client);
    //struct mma8653_axis *axis = (struct mma8653_axis *)rbuf;

	/* Report acceleration sensor information */
    input_report_abs(mma8653->input_dev, ABS_X, axis->x);
    input_report_abs(mma8653->input_dev, ABS_Y, axis->y);
    input_report_abs(mma8653->input_dev, ABS_Z, axis->z);
    input_sync(mma8653->input_dev);
    mmaprintkd("Gsensor x==%d  y==%d z==%d\n",axis->x,axis->y,axis->z);
}

/** \D4\DA \B5װ벿ִ\D0\D0, \BE\DF\CC\E5\BB\F1ȡ g sensor \CA\FD\BE\DD. */
static int mma8653_get_data(struct i2c_client *client)
{
    struct mma8653_data* mma8653 = i2c_get_clientdata(client);
	int ret;
	int x,y,z;
    struct mma8653_axis axis;
    struct gsensor_platform_data *pdata = pdata = client->dev.platform_data;

/* enabled only if FREAD MODE */
/*
	char buffer[3];
    do {
        memset(buffer, 0, 3);
        buffer[0] = MMA8653_REG_X_OUT_MSB;
        ret = mma8653_rx_data(client, &buffer[0], 3);
        if (ret < 0)
            return ret;
    } while (0);

	x = mma8653_convert_to_int(buffer[0],0);
	y = mma8653_convert_to_int(buffer[1],0);
	z = mma8653_convert_to_int(buffer[2],0);
*/
	char buffer[6];
 /*       
        buffer[0] = mma845x_read_reg(client, MMA8653_REG_X_OUT_MSB);
        buffer[1] = mma845x_read_reg(client, MMA8653_REG_X_OUT_LSB);
        buffer[2] = mma845x_read_reg(client, MMA8653_REG_Y_OUT_MSB);
        buffer[3] = mma845x_read_reg(client, MMA8653_REG_Y_OUT_LSB);
        buffer[4] = mma845x_read_reg(client, MMA8653_REG_Z_OUT_MSB);
        buffer[5] = mma845x_read_reg(client, MMA8653_REG_Z_OUT_LSB);

        x = (buffer[1] | (buffer[0] << 8)) >> 6;
        y = (buffer[3] | (buffer[2] << 8)) >> 6;
        z = (buffer[5] | (buffer[4] << 8)) >> 6;
        x = sign_extend32(x, 9);
        y = sign_extend32(y, 9);
        z = sign_extend32(z, 9);
*/

    do {
        memset(buffer, 0, 6);
        buffer[0] = MMA8653_REG_X_OUT_MSB;
        ret = mma8653_rx_data(client, &buffer[0], 6);
        if (ret < 0)
            return ret;
    } while (0);

	x = mma8653_convert_to_int(buffer[0],buffer[1]);
	y = mma8653_convert_to_int(buffer[2],buffer[3]);
	z = mma8653_convert_to_int(buffer[4],buffer[5]);

	if (pdata->swap_xyz) {
printk("lipf+++++++++++++++debug++++++++++++++!!!!!1111\n\n");
		axis.x = (pdata->orientation[0])*x + (pdata->orientation[1])*y + (pdata->orientation[2])*z;
		axis.y = (pdata->orientation[3])*x + (pdata->orientation[4])*y + (pdata->orientation[5])*z;	
		axis.z = (pdata->orientation[6])*x + (pdata->orientation[7])*y + (pdata->orientation[8])*z;
	}
	else {
		axis.x = -y;
		axis.y = -x;	
		axis.z = -z;
	}

	if(pdata->swap_xy)
	{
             axis.x = -axis.x;
             swap(axis.x,axis.y); 
	}
	
    mmaprintkd( "%s: ------------------mma8653_GetData axis = %d  %d  %d--------------\n",
            __func__, axis.x, axis.y, axis.z); 
     
    //memcpy(sense_data, &axis, sizeof(axis));
    mma8653_report_value(client, &axis);
	//atomic_set(&data_ready, 0);
	//wake_up(&data_ready_wq);

    /* \BB\A5\B3\E2\B5ػ\BA\B4\E6\CA\FD\BE\DD. */
    mutex_lock(&(mma8653->sense_data_mutex) );
    mma8653->sense_data = axis;
    mutex_unlock(&(mma8653->sense_data_mutex) );

    /* \D6\C3λ data_ready */
    atomic_set(&(mma8653->data_ready), 1);
    /* \BB\BD\D0\D1 data_ready \B5ȴ\FD\B6\D3\C1\D0ͷ. */
	wake_up(&(mma8653->data_ready_wq) );

	return 0;
}

/*
static int mma8653_trans_buff(char *rbuf, int size)
{
	//wait_event_interruptible_timeout(data_ready_wq,
	//				 atomic_read(&data_ready), 1000);
	wait_event_interruptible(data_ready_wq,
					 atomic_read(&data_ready));

	atomic_set(&data_ready, 0);
	memcpy(rbuf, &sense_data[0], size);

	return 0;
}
*/

static int mma8653_get_cached_data(struct i2c_client* client, struct mma8653_axis* sense_data)
{
    struct mma8653_data* this = (struct mma8653_data *)i2c_get_clientdata(client);

    wait_event_interruptible_timeout(this->data_ready_wq, 
                                     atomic_read(&(this->data_ready) ),
                                     msecs_to_jiffies(1000) );
    if ( 0 == atomic_read(&(this->data_ready) ) ) {
        printk("waiting 'data_ready_wq' timed out.");
        return -1;
    }
    mutex_lock(&(this->sense_data_mutex) );
    *sense_data = this->sense_data;
    mutex_unlock(&(this->sense_data_mutex) );
    return 0;
}

static int mma8653_open(struct inode *inode, struct file *file)
{
	return 0;//nonseekable_open(inode, file);
}

static int mma8653_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long mma8653_ioctl( struct file *file, unsigned int cmd,unsigned long arg)
{

	void __user *argp = (void __user *)arg;
	// char msg[RBUFF_SIZE + 1];
    struct mma8653_axis sense_data = {0};
	int ret = -1;
	char rate;
	struct i2c_client *client = container_of(mma8653_device.parent, struct i2c_client, dev);
    struct mma8653_data* this = (struct mma8653_data *)i2c_get_clientdata(client);  /* \C9豸\CA\FD\BE\DDʵ\C0\FD\B5\C4ָ\D5\EB. */

	switch (cmd) {
	case MMA_IOCTL_APP_SET_RATE:
		if (copy_from_user(&rate, argp, sizeof(rate)))
			return -EFAULT;
		break;
	default:
		break;
	}

	switch (cmd) {
	case MMA_IOCTL_START:
        mutex_lock(&(this->operation_mutex) );
        mmaprintkd("to perform 'MMA_IOCTL_START', former 'start_count' is %d.", this->start_count);
        (this->start_count)++;
        if ( 1 == this->start_count ) {
            atomic_set(&(this->data_ready), 0);
            if ( (ret = mma8653_start(client, MMA8653_RATE_12P5) ) < 0 ) {
                mutex_unlock(&(this->operation_mutex) );
                return ret;
            }
        }
        mutex_unlock(&(this->operation_mutex) );
        mmaprintkd("finish 'MMA_IOCTL_START', ret = %d.", ret);
        return 0;

	case MMA_IOCTL_CLOSE:
        mutex_lock(&(this->operation_mutex) );
        mmaprintkd("to perform 'MMA_IOCTL_CLOSE', former 'start_count' is %d, PID : %d", this->start_count, get_current()->pid);
        if ( 0 == (--(this->start_count) ) ) {
            atomic_set(&(this->data_ready), 0);
            if ( (ret = mma8653_close(client) ) < 0 ) {
                mutex_unlock(&(this->operation_mutex) );
                return ret;
            }
        }
        mutex_unlock(&(this->operation_mutex) );
        return 0;

	case MMA_IOCTL_APP_SET_RATE:
		ret = mma8653_reset_rate(client, rate);
		if (ret < 0)
			return ret;
		break;
	case MMA_IOCTL_GETDATA:
		// ret = mma8653_trans_buff(msg, RBUFF_SIZE);
		if ( (ret = mma8653_get_cached_data(client, &sense_data) ) < 0 ) {
            printk("failed to get cached sense data, ret = %d.", ret);
			return ret;
        }
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case MMA_IOCTL_GETDATA:
        /*
		if (copy_to_user(argp, &msg, sizeof(msg)))
			return -EFAULT;
        */
        if ( copy_to_user(argp, &sense_data, sizeof(sense_data) ) ) {
            printk("failed to copy sense data to user space.");
			return -EFAULT;
        }
		break;
	default:
		break;
	}

	return 0;
}

static void mma8653_work_func(struct work_struct *work)
{
	struct mma8653_data *mma8653 = container_of(work, struct mma8653_data, work);
	struct i2c_client *client = mma8653->client;
	
	if (mma8653_get_data(client) < 0) 
		mmaprintkd(KERN_ERR "MMA8653 mma_work_func: Get data failed\n");
		
	enable_irq(client->irq);		
}

static void  mma8653_delaywork_func(struct work_struct *work)
{
	struct delayed_work *delaywork = container_of(work, struct delayed_work, work);
	struct mma8653_data *mma8653 = container_of(delaywork, struct mma8653_data, delaywork);
	struct i2c_client *client = mma8653->client;

	if (mma8653_get_data(client) < 0) 
		printk(KERN_ERR "MMA8653 mma_work_func: Get data failed\n");
	mmaprintkd("%s :int src:0x%02x\n",__FUNCTION__,mma845x_read_reg(mma8653->client,MMA8653_REG_INTSRC));	
	enable_irq(client->irq);		
}

static irqreturn_t mma8653_interrupt(int irq, void *dev_id)
{
	struct mma8653_data *mma8653 = (struct mma8653_data *)dev_id;
	
	disable_irq_nosync(irq);
	schedule_delayed_work(&mma8653->delaywork, msecs_to_jiffies(30));
	mmaprintkf("%s :enter\n",__FUNCTION__);	
	return IRQ_HANDLED;
}

static struct file_operations mma8653_fops = {
	.owner = THIS_MODULE,
	.open = mma8653_open,
	.release = mma8653_release,
	.unlocked_ioctl = mma8653_ioctl,
};

static struct miscdevice mma8653_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mma8452_daemon",//"mma8653_daemon",
	.fops = &mma8653_fops,
};

static int mma8653_remove(struct i2c_client *client)
{
	struct mma8653_data *mma8653 = i2c_get_clientdata(client);
	
    misc_deregister(&mma8653_device);
    input_unregister_device(mma8653->input_dev);
    input_free_device(mma8653->input_dev);
    free_irq(client->irq, mma8653);
    kfree(mma8653); 
    this_client = NULL;
	return 0;
}

static const struct i2c_device_id mma8653_id[] = {
		{"gs_mma8653", 0},
		{ }
};

static struct i2c_driver mma8653_driver = {
	.driver = {
		.name = "gs_mma8653",
	    },
	.id_table 	= mma8653_id,
	.probe		= mma8653_probe,           
	.remove		= __devexit_p(mma8653_remove),
};


static int mma8653_init_client(struct i2c_client *client)
{
	struct mma8653_data *mma8653;
	int ret,irq;
	mma8653 = i2c_get_clientdata(client);
	mmaprintk("gpio_to_irq(%d) is %d\n",client->irq,gpio_to_irq(client->irq));
	if ( !gpio_is_valid(client->irq)) {
		mmaprintk("+++++++++++gpio_is_invalid\n");
		return -EINVAL;
	}
	ret = gpio_request(client->irq, "mma8653_int");
	if (ret) {
		mmaprintk( "failed to request mma7990_trig GPIO%d\n",gpio_to_irq(client->irq));
		return ret;
	}
    ret = gpio_direction_input(client->irq);
    if (ret) {
        mmaprintk("failed to set mma7990_trig GPIO gpio input\n");
		gpio_free(client->irq);
		return ret;
    }
	gpio_pull_updown(client->irq, GPIOPullUp);
	irq = gpio_to_irq(client->irq);
	ret = request_irq(irq, mma8653_interrupt, IRQF_TRIGGER_LOW | IRQF_ONESHOT, client->dev.driver->name, mma8653);
	mmaprintk("request irq is %d,ret is  0x%x\n",irq,ret);
	if (ret ) {
		gpio_free(client->irq);
		mmaprintk(KERN_ERR "mma8653_init_client: request irq failed,ret is %d\n",ret);
        return ret;
	}
	client->irq = irq;
	disable_irq(client->irq);
	init_waitqueue_head(&data_ready_wq);
 
	return 0;
}

static int  mma8653_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct mma8653_data *mma8653;
	struct gsensor_platform_data *pdata = pdata = client->dev.platform_data;
	int err;

	mmaprintkf("%s enter\n",__FUNCTION__);

	mma8653 = kzalloc(sizeof(struct mma8653_data), GFP_KERNEL);
	if (!mma8653) {
		mmaprintk("[mma8653]:alloc data failed.\n");
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
    
	INIT_WORK(&mma8653->work, mma8653_work_func);
	INIT_DELAYED_WORK(&mma8653->delaywork, mma8653_delaywork_func);

	memset(&(mma8653->sense_data), 0, sizeof(struct mma8653_axis) );
	mutex_init(&(mma8653->sense_data_mutex) );

	atomic_set(&(mma8653->data_ready), 0);
	init_waitqueue_head(&(mma8653->data_ready_wq) );

	mma8653->start_count = 0;
	mutex_init(&(mma8653->operation_mutex) );

	mma8653->status = MMA8653_CLOSE;

	mma8653->client = client;
	i2c_set_clientdata(client, mma8653);

	this_client = client;

	devid = mma8653_get_devid(this_client);
	if ((MMA8653_DEVID != devid) 
		&& (MMA8451_DEVID != devid)
		&& (MMA8453_DEVID != devid)) {
		pr_info("mma8653: invalid devid\n");
		goto exit_invalid_devid;
	}

	err = mma8653_init_client(client);
	if (err < 0) {
		mmaprintk(KERN_ERR
		       "mma8653_probe: mma8653_init_client failed\n");
		goto exit_request_gpio_irq_failed;
	}

	mma8653->input_dev = input_allocate_device();
	if (!mma8653->input_dev) {
		err = -ENOMEM;
		mmaprintk(KERN_ERR
		       "mma8653_probe: Failed to allocate input device\n");
		goto exit_input_allocate_device_failed;
	}

	set_bit(EV_ABS, mma8653->input_dev->evbit);

	/* x-axis acceleration */
	input_set_abs_params(mma8653->input_dev, ABS_X, -MMA845X_RANGE, MMA845X_RANGE, 0, 0); //2g full scale range
	/* y-axis acceleration */
	input_set_abs_params(mma8653->input_dev, ABS_Y, -MMA845X_RANGE, MMA845X_RANGE, 0, 0); //2g full scale range
	/* z-axis acceleration */
	input_set_abs_params(mma8653->input_dev, ABS_Z, -MMA845X_RANGE, MMA845X_RANGE, 0, 0); //2g full scale range

	// mma8653->input_dev->name = "compass";
	mma8653->input_dev->name = "gsensor";
	mma8653->input_dev->dev.parent = &client->dev;

	err = input_register_device(mma8653->input_dev);
	if (err < 0) {
		mmaprintk(KERN_ERR
		       "mma8653_probe: Unable to register input device: %s\n",
		       mma8653->input_dev->name);
		goto exit_input_register_device_failed;
	}

    mma8653_device.parent = &client->dev;
	err = misc_register(&mma8653_device);
	if (err < 0) {
		mmaprintk(KERN_ERR
		       "mma8653_probe: mmad_device register failed\n");
		goto exit_misc_device_register_mma8653_device_failed;
	}

	err = gsensor_sysfs_init();
	if (err < 0) {
		mmaprintk(KERN_ERR
            "mma8653_probe: gsensor sysfs init failed\n");
		goto exit_gsensor_sysfs_init_failed;
	}

	printk(KERN_INFO "mma8653 probe ok\n");
#if  0	
	//mma8653_start_test(this_client);
	mma8653_start(client, MMA8653_RATE_12P5);
#endif
	return 0;

exit_gsensor_sysfs_init_failed:
    misc_deregister(&mma8653_device);
exit_misc_device_register_mma8653_device_failed:
    input_unregister_device(mma8653->input_dev);
exit_input_register_device_failed:
	input_free_device(mma8653->input_dev);
exit_input_allocate_device_failed:
	free_irq(client->irq, mma8653);
exit_request_gpio_irq_failed:
	cancel_delayed_work_sync(&mma8653->delaywork);
	cancel_work_sync(&mma8653->work);
exit_invalid_devid:
	kfree(mma8653);	
exit_alloc_data_failed:
    ;
	mmaprintk("%s error\n",__FUNCTION__);
	return -1;
}


static int __init mma8653_i2c_init(void)
{
	return i2c_add_driver(&mma8653_driver);
}

static void __exit mma8653_i2c_exit(void)
{
	i2c_del_driver(&mma8653_driver);
}

module_init(mma8653_i2c_init);
module_exit(mma8653_i2c_exit);

