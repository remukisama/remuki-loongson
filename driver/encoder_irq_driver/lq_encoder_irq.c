#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define ENC_CMD_RST_CNT _IO('E', 1)
#define ENC_CMD_GET_DIR _IOR('E', 2, int)

#define MAX_ENCODERS 4

static int pinA[MAX_ENCODERS] = {[0 ... MAX_ENCODERS-1] = -1};
static int pinB[MAX_ENCODERS] = {[0 ... MAX_ENCODERS-1] = -1};
static int num_pinA = 0;
static int num_pinB = 0;

module_param_array(pinA, int, &num_pinA, S_IRUGO);
module_param_array(pinB, int, &num_pinB, S_IRUGO);

struct encoder_dev {
    int id;
    int pin_a;
    int pin_b;
    int irq_a;
    int irq_b;
    long long count;
    int dir;
    uint8_t last_state;
    spinlock_t lock;
    struct miscdevice miscdev;
    char name[32];
};

static struct encoder_dev my_encoders[MAX_ENCODERS];

static const int8_t state_table[16] = {
    0, -1,  1,  0, 
    1,  0,  0, -1, 
   -1,  0,  0,  1, 
    0,  1, -1,  0
};

static irqreturn_t encoder_isr(int irq, void *dev_id)
{
    struct encoder_dev *enc = (struct encoder_dev *)dev_id;
    uint8_t curr_state = 0;
    uint8_t index;
    int8_t delta;
    unsigned long flags;

    spin_lock_irqsave(&enc->lock, flags);
    if (gpio_get_value(enc->pin_a)) curr_state |= 0x01;
    if (gpio_get_value(enc->pin_b)) curr_state |= 0x02;

    if (curr_state != enc->last_state) {
        index = (enc->last_state << 2) | curr_state;
        delta = state_table[index];
        if (delta != 0) {
            enc->count += delta;
            enc->dir = delta;
        }
        enc->last_state = curr_state;
    }
    spin_unlock_irqrestore(&enc->lock, flags);

    return IRQ_HANDLED;
}

static ssize_t enc_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
    struct miscdevice *misc = file->private_data;
    struct encoder_dev *enc = container_of(misc, struct encoder_dev, miscdev);
    long long cnt;
    unsigned long flags;

    if (count < sizeof(cnt)) return -EINVAL;

    spin_lock_irqsave(&enc->lock, flags);
    cnt = enc->count;
    spin_unlock_irqrestore(&enc->lock, flags);

    if (copy_to_user(user_buf, &cnt, sizeof(cnt)))
        return -EFAULT;

    return sizeof(cnt);
}

static long enc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *misc = file->private_data;
    struct encoder_dev *enc = container_of(misc, struct encoder_dev, miscdev);
    unsigned long flags;
    int dir;

    switch (cmd) {
        case ENC_CMD_RST_CNT:
            spin_lock_irqsave(&enc->lock, flags);
            enc->count = 0;
            spin_unlock_irqrestore(&enc->lock, flags);
            break;
        case ENC_CMD_GET_DIR:
            spin_lock_irqsave(&enc->lock, flags);
            dir = enc->dir;
            spin_unlock_irqrestore(&enc->lock, flags);
            if (copy_to_user((int __user *)arg, &dir, sizeof(dir)))
                return -EFAULT;
            break;
        default:
            return -ENOTTY;
    }
    return 0;
}

static struct file_operations enc_fops = {
    .owner = THIS_MODULE,
    .read = enc_read,
    .unlocked_ioctl = enc_ioctl,
};

static int __init encoder_init(void)
{
    int i, ret;
    int valid_channels = 0;

    if (num_pinA == 0 || num_pinA != num_pinB) {
        printk(KERN_ERR "lq_encoder: pinA and pinB arrays must be provided and equal length.\n"
                        "Example 2 channels: insmod lq_encoder_irq.ko pinA=88,67 pinB=89,75\n");
        return -EINVAL;
    }

    for (i = 0; i < num_pinA && i < MAX_ENCODERS; i++) {
        struct encoder_dev *enc = &my_encoders[i];
        
        enc->id = i;
        enc->pin_a = pinA[i];
        enc->pin_b = pinB[i];
        enc->count = 0;
        enc->dir = 0;
        enc->last_state = 0;
        spin_lock_init(&enc->lock);

        snprintf(enc->name, sizeof(enc->name), "lq_encoder%d", i);

        ret = gpio_request(enc->pin_a, enc->name);
        if (ret) {
            printk(KERN_ERR "Failed to request pinA %d\n", enc->pin_a);
            continue;
        }
        gpio_direction_input(enc->pin_a);
        
        ret = gpio_request(enc->pin_b, enc->name);
        if (ret) {
            gpio_free(enc->pin_a);
            printk(KERN_ERR "Failed to request pinB %d\n", enc->pin_b);
            continue;
        }
        gpio_direction_input(enc->pin_b);

        if (gpio_get_value(enc->pin_a)) enc->last_state |= 0x01;
        if (gpio_get_value(enc->pin_b)) enc->last_state |= 0x02;

        enc->irq_a = gpio_to_irq(enc->pin_a);
        enc->irq_b = gpio_to_irq(enc->pin_b);

        ret = request_irq(enc->irq_a, encoder_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, enc->name, enc);
        if (ret) {
            gpio_free(enc->pin_a);
            gpio_free(enc->pin_b);
            continue;
        }

        ret = request_irq(enc->irq_b, encoder_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, enc->name, enc);
        if (ret) {
            free_irq(enc->irq_a, enc);
            gpio_free(enc->pin_a);
            gpio_free(enc->pin_b);
            continue;
        }

        enc->miscdev.minor = MISC_DYNAMIC_MINOR;
        enc->miscdev.name = enc->name;
        enc->miscdev.fops = &enc_fops;

        ret = misc_register(&enc->miscdev);
        if (ret) {
            free_irq(enc->irq_b, enc);
            free_irq(enc->irq_a, enc);
            gpio_free(enc->pin_a);
            gpio_free(enc->pin_b);
            continue;
        }
        printk(KERN_INFO "lq_encoder%d: initialized on pins %d and %d\n", i, enc->pin_a, enc->pin_b);
        valid_channels++;
    }

    return valid_channels > 0 ? 0 : -ENODEV;
}

static void __exit encoder_exit(void)
{
    int i;
    for (i = 0; i < num_pinA && i < MAX_ENCODERS; i++) {
        struct encoder_dev *enc = &my_encoders[i];
        if (enc->irq_a > 0) {
            misc_deregister(&enc->miscdev);
            free_irq(enc->irq_b, enc);
            free_irq(enc->irq_a, enc);
            gpio_free(enc->pin_b);
            gpio_free(enc->pin_a);
            enc->irq_a = 0;
        }
    }
    printk(KERN_INFO "lq_encoder: exited\n");
}

module_init(encoder_init);
module_exit(encoder_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("AI");
MODULE_DESCRIPTION("Multi-channel Quadrature Encoder Interrupt Driver");
