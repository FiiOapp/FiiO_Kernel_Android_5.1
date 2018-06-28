#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/cpufreq.h>

static char gov_file[]="/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor";
static char speed_file[]="/sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed";
static char buf[16] = "";
static char buf1[16];
/* "interactive conservative ondemand powersave userspace performance " */

extern int gpolicy_lock;
extern struct kobject *cpufreq_global_kobject;
extern int __cpufreq_set_policy(struct cpufreq_policy *data,
				struct cpufreq_policy *policy);


/* This function is used to lock cpu freq
 * gov  : !0 to lock policy 
 * min   cpu min freq
 * max   cpu max freq
 *
 */
int cpufreq_set_policy(char gov,int min,int max)
{


	unsigned int ret = -EINVAL;
	struct cpufreq_policy new_policy;
	struct cpufreq_policy *policy;
	policy = cpufreq_cpu_get(0);
	if (!policy)
		goto no_policy;

	ret = cpufreq_get_policy(&new_policy, policy->cpu);
	if (ret)
		return -EINVAL;

    new_policy.min=min;
    new_policy.max=max;

    // unlock policy
    if(!gov)
        gpolicy_lock=gov;

	ret = __cpufreq_set_policy(policy, &new_policy);

    gpolicy_lock=gov;

	policy->user_policy.min= policy->min;
	policy->user_policy.max= policy->max;

	return ret ;

no_policy:

    return ret;
}





int cpufreq_set_governor_sys(char gov,int freq)
{
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;
    int freqs=0;

    switch(gov){
    case 'i':
         strcpy(buf,"interactive");break;
    case 'u':
         strcpy(buf,"userspace");
         freqs= freq? freq:816000;
         break;

    default:
         strcpy(buf,"interactive");break;
    }
    printk("gov is %s freqs %d\n",buf,freqs);

    fp = filp_open(gov_file, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(fp)) {
        printk("open gov file error %d \n",fp);
        return -1;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_write(fp, buf, sizeof(buf), &pos);
    pos = 0;
    vfs_read(fp, buf1, sizeof(buf), &pos);
    printk("gov read: %s\n", buf1);
    filp_close(fp, NULL);
    set_fs(fs);

    sprintf(buf,"%d",freqs);
    fp = filp_open(speed_file, O_RDWR | O_CREAT, 0666);
    if (IS_ERR(fp)) {
        printk("open gov speed file error %d \n",fp);
        return -1;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_write(fp, buf, sizeof(buf), &pos);
    pos = 0;
    vfs_read(fp, buf1, sizeof(buf), &pos);
    printk("gov speed read: %s\n", buf1);
    filp_close(fp, NULL);
    set_fs(fs);

    return 0;
}
