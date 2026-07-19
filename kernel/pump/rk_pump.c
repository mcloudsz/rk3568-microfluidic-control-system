#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>               
#include <linux/mod_devicetable.h>  
#include <linux/fs.h>
#include <linux/pwm.h>
#include <linux/rk_pump.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#define PUMP_NR 3  // 泵的总个数: 两个伺服电机驱动泵 + 一个直流电机驱动泵

typedef enum pump_type {
    MOTOR_TYPE_DC = 0,
    MOTOR_TYPE_SERVO,
}pump_type_t;

// 阀描述符
typedef struct valve_desc
{
    struct gpio_desc *valve_gpio;
    bool is_open;
}valve_desc_t;

// 泵 + 阀描述符
typedef struct pump_desc
{
    pump_type_t type;  // 类型 MOTOR_TYPE_DC 直流泵 / MOTOR_TYPE_SERVO 伺服泵
    u8 idx;   // 下标
    union
    {
        // 如果是伺服电机, 需要管理三个引脚
        struct {
            struct gpio_desc *pump_dir_gpio;
            struct gpio_desc *pump_ena_gpio;
        };
        // 如果是直流电机, 直接控制两个引脚就行
        struct{
            struct gpio_desc *pump_pos_gpio; 
            struct gpio_desc *pump_neg_gpio;
        };
    };
    
    struct pwm_device * pump_pwm;      // pwm 资源
    valve_desc_t valve;
    int freq_hz;   // 当前的频率
}pump_desc_t;


/* 驱动状态机
SERVO_IDLE   -> SERVO_MIXING
SERVO_MIXING -> SERVO_IDLE
SERVO_IDLE   -> SERVO_FLUSHING
SERVO_IDLE   -> SERVO_IDLE
SERVO_FLUSHING -> SERVO_IDLE
*/

typedef enum servo_status
{
    SERVO_IDLE = 0,
    SERVO_MIXING,
    SERVO_FLUSHING,
}servo_status_t;

// 字符驱动私有数据
typedef struct servo_desc
{
    pump_desc_t* pump_descs;  // 内嵌
    u8 pump_nr;
    struct miscdevice misc_dev; // 对应的 misc deivce 句柄. 三组泵/阀共用一个设备
    servo_status_t status;  // 状态标志位
    atomic_t opened;     // 标志位, 防止多次 open
    struct mutex lock;   // 设备锁
}servo_desc_t;

// 设备树资源
typedef struct resources
{
    u8 type;
    const char *pul_gpio_id;
    const char *dir_gpio_id;
    const char *ena_gpio_id;
    const char *valve_gpio_id;
    const char *pwm_id;
}resources_t;


// 静态定义的设备树资源结构体
static resources_t of_resources[PUMP_NR] = {
    {MOTOR_TYPE_SERVO, "pul0", "dir0", "ena0", "valve0", "pwm0"},
    {MOTOR_TYPE_SERVO, "pul1", "dir1", "ena1", "valve1", "pwm1"},
    {MOTOR_TYPE_DC, "in_pos2", "in_neg2", NULL, "valve2", NULL},
};

static const struct file_operations pump_misc_fops;

static pump_desc_t pump_descs[PUMP_NR] = {
    [0] = {
        .type          = MOTOR_TYPE_SERVO,
        .idx           = 0,
        .pump_dir_gpio = NULL,
        .pump_ena_gpio = NULL,
        .pump_pwm      = NULL,
        .valve = {
            .valve_gpio = NULL,
            .is_open = false,
        },
        .freq_hz = 0,
    },
    [1] = {
        .type          = MOTOR_TYPE_SERVO,
        .idx           = 1,
        .pump_dir_gpio = NULL,
        .pump_ena_gpio = NULL,
        .pump_pwm      = NULL,
        .valve = {
            .valve_gpio = NULL,
            .is_open = false,
        },
        .freq_hz = 0,
    },
    [2] = {
        .type          = MOTOR_TYPE_DC,
        .idx           = 2,
        .pump_pos_gpio = NULL, 
        .pump_neg_gpio = NULL,
        .pump_pwm      = NULL,
        .valve = {
            .valve_gpio = NULL,
            .is_open = false,
        },
        .freq_hz = 0,  // 无意义
    }
};

static servo_desc_t servo_desc = {
    .pump_descs = pump_descs,
    .pump_nr = PUMP_NR,
    .misc_dev = {
        .minor = MISC_DYNAMIC_MINOR, 
        .name  = "rk_pump",
        .fops  = &pump_misc_fops, 
    },
    .status = SERVO_IDLE,
    .opened = ATOMIC_INIT(0),  //true
};

// 规则: 先打开阀 -> 再打开泵  先关闭泵 -> 再关闭阀
// 开启阀的底层函数
static void valve_on(valve_desc_t *valve)
{
    gpiod_set_value(valve->valve_gpio, 1); 
    msleep(20);
}

// 关闭阀的底层函数
static void valve_off(valve_desc_t *valve)
{
    gpiod_set_value(valve->valve_gpio, 0); 
    msleep(20);
}

static int pump_set_freq(pump_desc_t *pump, int freq_hz);
// 开启泵 + 阀组合的底层函数
static int servo_pump_on(pump_desc_t *pump)
{
    // 必须先打开阀
    if(!pump || pump->type != MOTOR_TYPE_SERVO)
        return -EINVAL;

    valve_on(&pump->valve);  // 不可能失败. 因为检查过 pump->valve 指针不为空

    // 打开阀之后, 再打开泵.
    // 检查, pump 是否注册了 pwm 资源
    if (!pump->pump_pwm)
    {
        valve_off(&pump->valve);
        return -EINVAL;
    }

    // 使能 GPIO
    gpiod_set_value(pump->pump_ena_gpio, 1); // 使能

    int ret = pump_set_freq(pump, DEFAULT_PUMP_FREQ);
    if(ret)
        return ret;

    // 使能 pwm
    ret = pwm_enable(pump->pump_pwm);
    if(ret)
        return ret;
    return 0;
}

// 关闭泵 + 阀组合的底层函数
static int servo_pump_off(pump_desc_t *pump)
{
    if(!pump || !pump->pump_pwm)
        return -EINVAL;
    if(pump->type != MOTOR_TYPE_SERVO)
        return -EINVAL;

    // 失能 pwm
    pwm_disable(pump->pump_pwm);
    // 失能 GPIO
    gpiod_set_value(pump->pump_ena_gpio, 0); 

    // 后关闭泵
    valve_off(&pump->valve);  // 不可能失败. 因为检查过 pump->valve 指针不为空
    pump->freq_hz = 0;
    return 0;
}

// 开启直流泵 + 阀组合的底层函数
static int dc_pump_on(pump_desc_t *pump)
{
    if(!pump || pump->type != MOTOR_TYPE_DC)
        return -EINVAL;

    valve_on(&pump->valve);
    gpiod_set_value(pump->pump_pos_gpio, 1);
    gpiod_set_value(pump->pump_neg_gpio, 0);
    return 0;
}

// 关闭直流泵 + 阀组合的底层函数
static int dc_pump_off(pump_desc_t *pump)
{
    if(!pump || pump->type != MOTOR_TYPE_DC)
        return -EINVAL;

    gpiod_set_value(pump->pump_pos_gpio, 0);
    gpiod_set_value(pump->pump_neg_gpio, 0);
    valve_off(&pump->valve);

    return 0;
}

// 设置 pump 的脉冲频率
static int pump_set_freq(pump_desc_t *pump, int freq_hz)
{
    unsigned long duty_ns, period_ns;

    if (!pump || !pump->pump_pwm)
        return -EINVAL;

    if (freq_hz <= 0)
        return -EINVAL;

    // 计算 PWM 周期和占空比（50% 占空比）
    period_ns = 1000000000UL / freq_hz; // 周期 ns
    duty_ns   = period_ns / 2;          // 50% 占空比

    // 配置 PWM
    int ret = pwm_config(pump->pump_pwm, duty_ns, period_ns);
    if (ret)
        return ret;

    pump->freq_hz = freq_hz;
    return 0;
}

// 字符设备打开回调
static int pump_mdev_open(struct inode *inode, struct file *file)
{
    // 从 file->priv 中获取 mdev
    struct miscdevice *mdev = file->private_data;

    // 从 mdev 中获取驱动侧结构体 servo_desc_t
    servo_desc_t *servo = container_of(mdev, servo_desc_t, misc_dev);

    // 已被打开
    if(atomic_cmpxchg(&servo->opened, 0, 1) != 0)
    {
        pr_warn("pump device is already opened\n");
        return -EBUSY;
    }
    // 直接使用, 不做失败处理
    // 挂载 pump_descs[i] 到 file 的私有数据中
    file->private_data = (void*)servo;
    pr_info("pump_mdev_open success\n");
    return 0;
}

// 字符设备关闭回调
static int cmd_emergency_stop(void);
static int pump_mdev_release(struct inode *inode, struct file *file)
{
    servo_desc_t *servo = file->private_data;

    // 加锁
    mutex_lock(&servo->lock);
    cmd_emergency_stop();       // 急停
    servo->status = SERVO_IDLE;  // 重置回初始状态
    atomic_set(&servo->opened, 0);  // 设置 open 标志位为 false
    mutex_unlock(&servo->lock);

	pr_info("pump_mdev_release\n");
    return 0;
}

// 启动混合：血液泵(0) + 缓冲液泵(1)
static int cmd_start_mix(void)
{
    int ret;
    ret = servo_pump_on(&pump_descs[0]);
    if (ret)
        return ret;
    ret = servo_pump_on(&pump_descs[1]);
    if (ret) {
        servo_pump_off(&pump_descs[0]);  /* 回滚 */
        return ret;
    }
    return 0;
}

// 停止混合：安全关闭血液泵(0) + 缓冲液泵(1)
static int cmd_stop_mix(void)
{
    int ret0 = servo_pump_off(&pump_descs[0]);
    int ret1 = servo_pump_off(&pump_descs[1]);
    return ret0 ? ret0 : ret1;  // 两个都尝试关闭，返回第一个错误 
}

// 启动冲洗：先停混合，再开冲洗泵(2)
static int cmd_flush(void)
{
    int ret = 0;
    // 关闭
    ret = servo_pump_off(&pump_descs[0]);
    if(ret)
        return ret;
    ret = servo_pump_off(&pump_descs[1]);
    if(ret)
        return ret;
    return dc_pump_on(&pump_descs[2]);
}

// 冲洗完毕：关冲洗泵(2)
static int cmd_flush_done(void)
{
    return dc_pump_off(&pump_descs[2]);
}

// 紧急停止：关闭所有泵和阀
static int cmd_emergency_stop(void)
{
    servo_pump_off(&pump_descs[0]);
    servo_pump_off(&pump_descs[1]);
    dc_pump_off(&pump_descs[2]);
    return 0;
}

static long do_pump_mdev_ioctl(servo_desc_t *servo, unsigned int cmd, unsigned long arg)
{
    if(servo == NULL)
        return -EFAULT;

    struct pump_freq freq_arg;
    int ret = 0;

    if (_IOC_TYPE(cmd) != PUMP_IOC_MAGIC)
        return -ENOTTY;
        
    switch (cmd) {
    case PUMP_CMD_START_MIX:
        // 1. 禁止重复开启 2.flushing 未完成, 禁止开启 3. 其余状态均可开启, 开启后进入 MIXING 状态机
        if(servo->status == SERVO_MIXING || servo->status == SERVO_FLUSHING)
            return -EINVAL;
        ret = cmd_start_mix();
        if(ret == 0)
            servo->status = SERVO_MIXING;
        break;

    case PUMP_CMD_STOP_MIX:
        if(servo->status != SERVO_MIXING)  // 必须在 MIXING 状态下才能停止
            return -EINVAL;
        ret = cmd_stop_mix();
        if(ret == 0)
            servo->status = SERVO_IDLE;  // 急停
        break;

    case PUMP_CMD_FLUSH:
        // 1. 禁止重复冲洗
        if(servo->status == SERVO_FLUSHING)
            return -EINVAL;
        ret = cmd_flush();
        if(ret == 0)
            servo->status = SERVO_FLUSHING;  // 正在冲洗状态
        break;

    case PUMP_CMD_FLUSH_DONE:
        // 1. 只有在冲洗状态才会进入这个分支
        if(servo->status != SERVO_FLUSHING)
            return -EINVAL;
        ret = cmd_flush_done();
        if(ret == 0)
            servo->status = SERVO_IDLE;  // IDLE 状态
        break;

    case PUMP_CMD_EMERGENCY_STOP:
        ret = cmd_emergency_stop();  // 任何情况下都可调用
        if(ret == 0)
            servo->status = SERVO_IDLE;  // 急停
        break;

    // 低层命令：针对单泵的参数调节
    case PUMP_SET_FREQ:
        if (copy_from_user(&freq_arg, (void __user *)arg, sizeof(freq_arg)))
            return -EFAULT;

        // 获取 pump 编号, 检查. pump_id !=2, 因为 id 为 2 的泵是直流泵, 禁止调节
        u8 pump_id = freq_arg.pump_id;
        if (pump_id >= PUMP_NR || pump_id == 2)
            return -EINVAL;

        // 检查 pump 类型(防御)
        if (servo->pump_descs[pump_id].type != MOTOR_TYPE_SERVO)
            return -EINVAL;

        // 检查状态. 只有在 MIXING 状态下才允许参数调节
        if (servo->status != SERVO_MIXING) 
              return -EPERM;

        // 检查通过之后, 进行频率调节
        ret = pump_set_freq(&servo->pump_descs[pump_id], freq_arg.freq_hz);
        break;

    case PUMP_GET_STATUS:
        if (copy_to_user((void __user *)arg, &servo->status, sizeof(servo->status)))
            return -EFAULT;
        break;

    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

static long pump_mdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    servo_desc_t *servo = (servo_desc_t *)file->private_data;
    int ret = 0;
    mutex_lock(&servo->lock);
    ret = do_pump_mdev_ioctl(servo, cmd, arg);
    mutex_unlock(&servo->lock);
    return ret;
}


static const struct file_operations pump_misc_fops = {
    .owner          = THIS_MODULE,
    .open           = pump_mdev_open,
    .release        = pump_mdev_release,
    .unlocked_ioctl = pump_mdev_ioctl,
};


static int rk_pump_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    dev_info(dev, "\n\n============================================\n");
    dev_info(dev, "pump probe begin\n");
    dev_info(dev, "============================================\n\n");

    // 1. 获取设备侧资源, 包括：GPIO、PWM.
    int ret = 0;
    for(int i = 0; i < PUMP_NR; i++)
    {
        // 直流泵需要获取正向控制 GPIO.
        // 伺服泵的 PUL 引脚由 PWM 控制，不再重复申请为 GPIO
        if(of_resources[i].type == MOTOR_TYPE_DC)
        {
            struct gpio_desc *pos_gpio = devm_gpiod_get(dev, of_resources[i].pul_gpio_id, GPIOD_OUT_LOW);
            if(IS_ERR(pos_gpio))
            {
                dev_err(dev, "failed to get gpio(%s)\n", of_resources[i].pul_gpio_id);
                return dev_err_probe(dev, PTR_ERR(pos_gpio), "failed to get gpio(%s)\n", of_resources[i].pul_gpio_id);
            }
            pump_descs[i].pump_pos_gpio = pos_gpio;
        }

        // 获取 dir gpio / in gpio 资源, 默认设置为高电平
        struct gpio_desc * dir_gpio = devm_gpiod_get(dev, of_resources[i].dir_gpio_id, GPIOD_OUT_LOW);
        if(IS_ERR(dir_gpio))
        {
            dev_err(dev, "failed to get gpio(%s)\n", of_resources[i].dir_gpio_id);
            return dev_err_probe(dev, PTR_ERR(dir_gpio), "failed to get gpio(%s)\n", of_resources[i].dir_gpio_id);
        }
        pump_descs[i].pump_dir_gpio = dir_gpio;

        // 获取电磁阀 gpio 资源.
        struct gpio_desc * valve_gpio = devm_gpiod_get(dev, of_resources[i].valve_gpio_id, GPIOD_OUT_LOW);
        if(IS_ERR(valve_gpio))
        {
            dev_err(dev, "failed to get gpio(%s)\n", of_resources[i].valve_gpio_id);
            return dev_err_probe(dev, PTR_ERR(valve_gpio), "failed to get gpio(%s)\n", of_resources[i].valve_gpio_id);
        }
        pump_descs[i].valve.valve_gpio = valve_gpio;

        // 对于伺服阀, 获取 ENA GPIO 资源, 获取 pwm 资源
        if(of_resources[i].type == MOTOR_TYPE_SERVO)
        {
            // GPIO
            struct gpio_desc * ena_gpio = devm_gpiod_get(dev, of_resources[i].ena_gpio_id, GPIOD_OUT_LOW);
            if(IS_ERR(ena_gpio))
            {
                dev_err(dev, "failed to get gpio(%s)\n", of_resources[i].ena_gpio_id);
                return dev_err_probe(dev, PTR_ERR(ena_gpio), "failed to get gpio(%s)\n", of_resources[i].ena_gpio_id);
            }
            pump_descs[i].pump_ena_gpio = ena_gpio;

            // PWM
            struct pwm_device* pwm = devm_pwm_get(dev, of_resources[i].pwm_id);
            if(IS_ERR(pwm))
            {
                dev_err(dev, "failed to get pwm(%s)\n", of_resources[i].pwm_id);
                return dev_err_probe(dev, PTR_ERR(pwm), "failed to get pwm(%s)\n", of_resources[i].pwm_id);
            }
            pump_descs[i].pump_pwm = pwm;
        }
    }

    // 初始化锁
    mutex_init(&servo_desc.lock);

    // 2. 注册为 misc 设备
    ret = misc_register(&servo_desc.misc_dev);
    if (ret) 
    {
        dev_err(&pdev->dev, "failed to register misc device\n");
        return ret;
    }
    dev_info(dev, "rk_pump probe finish\n");

    return 0;
}

static int rk_pump_remove(struct platform_device *pdev)
{
    // 注销 misc 设备
    misc_deregister(&servo_desc.misc_dev);

    // 加锁
    mutex_lock(&servo_desc.lock);
    // 硬件安全关闭
    cmd_emergency_stop();
    servo_desc.status = SERVO_IDLE;  // 修改标志位
    atomic_set(&servo_desc.opened, 0);
    // 解锁
    mutex_unlock(&servo_desc.lock);

    mutex_destroy(&servo_desc.lock);  // 销毁锁
    dev_info(&pdev->dev, "rk_pump removed\n");
    return 0;
}

static const struct of_device_id pump_of_match[] = {
    { .compatible = "rk3568-project,rk_pump" }, 
    { }
};
MODULE_DEVICE_TABLE(of, pump_of_match);


static struct platform_driver rk_pump_driver = {
    .probe  = rk_pump_probe,
    .remove = rk_pump_remove,
    .driver = {
        .name = "rk3568_motor_driver",
        .of_match_table = pump_of_match, 
    },
};

module_platform_driver(rk_pump_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Rockchip Pump Misc Device");
MODULE_AUTHOR("mclouds");
