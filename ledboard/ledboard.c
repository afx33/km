/*
 * ledboard.c - ledboard gpio support (piborg.org)
 *
 * author: afx33sd
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/printk.h>
#include <linux/init.h>
#include <linux/module.h>

#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/ioport.h>

#include <asm/io.h>

#define LEDBRD_DEBUG

#ifdef LEDBRD_DEBUG
	#define LEDBRD_ROUTINE_ENTER	pr_notice(">> %s\n", __FUNCTION__);
	#define LEDBRD_ROUTINE_EXIT	pr_notice("%s <<\n", __FUNCTION__);
	#define LEDBRD_DBG_PR(fmt, ...)	pr_notice(fmt, ##__VA_ARGS__ );
#else
	#define LEDBRD_ROUTINE_ENTER	
	#define LEDBRD_ROUTINE_EXIT
	#define LEDBRD_DBG_PR(fmt, ...)
		
#endif

#define CHCK_LAMP_RANGE(x, min, max)	((x) < (min) ? (min) : (x) > (max) ? (max) : (x))
#define MAX_LAMP_VAL 1
#define MIN_LAMP_VAL 0
#define CHCK_LAMP_VAL(val) CHCK_LAMP_RANGE(val, MIN_LAMP_VAL, MAX_LAMP_VAL)

/*
 *from: /proc/iomem
 *
 20200000-20200fff : bcm2708_gpio
*/

/*
 * from: arch/arm/boot/dts/bcm2835.dtsi
 *
  gpio: gpio@7e200000 {
	   compatible = "brcm,bcm2835-gpio";
	   reg = <0x7e200000 0xb4>;
	   interrupts = <2 17>, <2 18>, <2 19>, <2 20>;

   gpio-controller;
   #gpio-cells = <2>;

   interrupt-controller;
   #interrupt-cells = <2>;
   }
*/

/*
 * GPIO 17	0	Red on/off
 * GPIO 27 	2	Green on/off
 * GPIO 22	3	Blue on/off
 */

#define GPIO_PIN_RED 17
#define GPIO_PIN_GREEN 27
#define GPIO_PIN_BLUE 22

#define GPIO_ADDR (0x20200000)
#define GPIO_SIZE (0xb4)

#define LEDBOARD_SYM_NAME "ledboard"

#define GPFSEL0_ADDR(gpio_addr) (gpio_addr)
#define GPFSEL1_ADDR(gpio_addr) (gpio_addr + 0x04)
#define GPFSEL2_ADDR(gpio_addr) (gpio_addr + 0x08)

#define GPFSEL_ADDR(gpio_addr, pin) (((pin/10) == 1) ? GPFSEL1_ADDR(gpio_addr) : GPFSEL2_ADDR(gpio_addr))
#define GPFSEL_OUTPUT(reg, pin) ((~(0x7<<((pin%10)*3)) & reg) | (0x1<<(pin%10)*3))

#define GPFSET0_ADDR(gpio_addr) (gpio_addr + 0x1C)
#define GPCLR0_ADDR(gpio_addr) (gpio_addr + 0x28)
#define GPCLR1_ADDR(gpio_addr) (gpio_addr + 0x2C)

// 
#define GPCLR_REG(pin, reg) ((0x1<<(pin)) | reg)
#define GPFSET_ON_REG(pin, reg) ((0x1<<(pin)) | reg)
#define GPFSET_OFF_REG(pin, reg) (~((0x1<<(pin))) & reg)

// GPFSETn and GPCLRn are used for avoiding of reading full register content 
// to perform off or on operation under pin
#define GPFSET_ON(pin)	(0x1<<(pin))
#define GPCLR(pin) (0x1<<(pin))

#define LEDBOARD_NUM_PARAMS 3

void ledboard_clear_pin(u8 pin);
void ledboard_set_pin_on(u8 pin);
void ledboard_set_pin(u8 pin);
void ledboard_set_pin_off(u8 pin);
void ledboard_output_pin(u8 pin);

int ledboard_open(struct inode *inode, struct file *filp);
int ledboard_release(struct inode *inode, struct file *filp);
ssize_t ledboard_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
ssize_t ledboard_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos);

struct led_ctrl {
	int red;
	int green;
	int blue;
};

struct led {
	dev_t dev;
	struct cdev cdev;	// symbol device structure
	struct semaphore sem;
	struct resource *gpio_res; 
	void *gpio_addr;	// don't use this directly !!
	struct led_ctrl ctrl;
} led_dev;

struct file_operations led_fops = {
	.owner		= THIS_MODULE,
	.read		= ledboard_read,
	.write		= ledboard_write,
	.open		= ledboard_open,
	.release	= ledboard_release,
};

MODULE_LICENSE("Dual BSD/GPL");

static int ledboard_init(void)
{
    	int err;	
	int devno;

	LEDBRD_ROUTINE_ENTER

	//pr_alert("Hello, ledboard!!\n");
	
	if ((err = alloc_chrdev_region(&led_dev.dev, 0, 1, LEDBOARD_SYM_NAME))) {
		pr_notice("failed to allocate character device: error(%de)\n", err);
		goto bad_alloc;
	}

	cdev_init(&led_dev.cdev, &led_fops);

	led_dev.cdev.owner = THIS_MODULE;
	led_dev.cdev.ops = &led_fops;

	devno = MKDEV(MAJOR(led_dev.dev), MINOR(led_dev.dev));

	led_dev.gpio_res = request_mem_region(GPIO_ADDR, GPIO_SIZE, LEDBOARD_SYM_NAME);
	if (!led_dev.gpio_res) {
		pr_notice("failed to request memory region: address(%d) size(%d)\n", GPIO_ADDR, GPIO_SIZE);
		goto bad_req_mem;
	}

	led_dev.gpio_addr = ioremap(GPIO_ADDR, GPIO_SIZE);
	if (led_dev.gpio_addr == NULL) {
		pr_notice("failed to map gpio address\n");
		goto bad_map;
	}
	
	LEDBRD_DBG_PR("gpio_addr: 0x%p\n", led_dev.gpio_addr);

	ledboard_output_pin(GPIO_PIN_RED);
	ledboard_output_pin(GPIO_PIN_GREEN);
	ledboard_output_pin(GPIO_PIN_BLUE);

	ledboard_clear_pin(GPIO_PIN_RED);
	ledboard_clear_pin(GPIO_PIN_GREEN);
	ledboard_clear_pin(GPIO_PIN_BLUE);
	
	led_dev.ctrl.red = led_dev.ctrl.blue = led_dev.ctrl.green = 0;
	
	sema_init(&led_dev.sem, 1);

	// warn: we can't call cdev_add before device will be well prepared,
	// because the device may be used by anybody
	if ((err = cdev_add(&led_dev.cdev, devno, 1))) {
		pr_notice("failed to add device: error(%de)\n", err);
		goto bad_add;
	}

	LEDBRD_ROUTINE_EXIT

	return 0;

bad_add:
	iounmap(led_dev.gpio_addr);

bad_map:
	release_mem_region(GPIO_ADDR, GPIO_SIZE);

bad_req_mem:
	unregister_chrdev_region(led_dev.dev, 1);

bad_alloc:
	return err;
}

static void ledboard_exit(void)
{
	LEDBRD_ROUTINE_ENTER

	ledboard_clear_pin(GPIO_PIN_RED);
	ledboard_clear_pin(GPIO_PIN_GREEN);
	ledboard_clear_pin(GPIO_PIN_BLUE);
		
	cdev_del(&led_dev.cdev);

	iounmap(led_dev.gpio_addr);
	release_mem_region(GPIO_ADDR, GPIO_SIZE);
	unregister_chrdev_region(led_dev.dev, 1);

	//pr_alert("Goodbye, ledboard!!\n");

	LEDBRD_ROUTINE_EXIT
}


int ledboard_open(struct inode *inode, struct file *filp)
{
	LEDBRD_ROUTINE_ENTER

	LEDBRD_ROUTINE_EXIT
	return 0;
}

int ledboard_release(struct inode *inode, struct file *filp)
{
	LEDBRD_ROUTINE_ENTER

	LEDBRD_ROUTINE_EXIT
	return 0;
}

ssize_t ledboard_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	size_t ret = 0;
	int i;
	char data;

	LEDBRD_ROUTINE_ENTER

	if (down_interruptible(&led_dev.sem)) 
		return -ERESTARTSYS;

	if (*f_pos >= LEDBOARD_NUM_PARAMS) goto out;
	if (*f_pos + count > LEDBOARD_NUM_PARAMS) count = LEDBOARD_NUM_PARAMS - *f_pos;

	for (i = 0; i < count; ++i) {
		switch (*f_pos) {
			case 0:		data = '0' + (char)led_dev.ctrl.red;	break;
			case 1:		data = '0' + (char)led_dev.ctrl.green;	break;
			case 2:		data = '0' + (char)led_dev.ctrl.blue;	break;
			default:	data = '\0';				break;
		}	
		if (copy_to_user(buf + i, &data, sizeof(data))) {
			ret = -EFAULT;
			goto out;
		}
		++*f_pos;
		++ret;
	}

out:
	up(&led_dev.sem);

	LEDBRD_ROUTINE_EXIT

	return ret;
}

//
// format:
// <RED_VAL><GREEN_VAL><BLUE_VAL>
// <value>: 0 or 1 in ascii (corresponds to OFF and ON)

ssize_t ledboard_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	ssize_t ret = 0;
	char data;
	int i;

	LEDBRD_ROUTINE_ENTER

	if (down_interruptible(&led_dev.sem))
		return -ERESTARTSYS;

	for (i = 0; i < count; ++i) {
		if (copy_from_user(&data, buf + i, sizeof(data))) {
			ret = -EFAULT;
			goto out;
		}
		
		LEDBRD_DBG_PR("i=%d f_pos=%lld data=%c ret=%d\n", i, *f_pos, data, ret);

		switch (*f_pos) {
			case 0:	
				led_dev.ctrl.red	  = CHCK_LAMP_VAL(data - '0');		
				if (led_dev.ctrl.red == 0) {
					ledboard_clear_pin(GPIO_PIN_RED);
				} else {
					ledboard_set_pin(GPIO_PIN_RED);
				}
				break;
			case 1:	
				led_dev.ctrl.green    = CHCK_LAMP_VAL(data - '0');		
				if (led_dev.ctrl.green == 0) {
					ledboard_clear_pin(GPIO_PIN_GREEN);
				} else {
					ledboard_set_pin(GPIO_PIN_GREEN);
				}
				break;
			case 2:	
				led_dev.ctrl.blue     = CHCK_LAMP_VAL(data - '0');		
				if (led_dev.ctrl.blue == 0) {
					ledboard_clear_pin(GPIO_PIN_BLUE);
				} else {
					ledboard_set_pin(GPIO_PIN_BLUE);
				}
				break;
			default:
				LEDBRD_DBG_PR("unexpected data=%x\n", data);
				break;
		}	
		++*f_pos;
		++ret;
	}
out:
	up(&led_dev.sem);

	LEDBRD_ROUTINE_EXIT

	return ret;
}

void ledboard_output_pin(u8 pin)
{
	unsigned int reg;

	unsigned int tmp;

	LEDBRD_ROUTINE_ENTER

	reg = ioread32(GPFSEL_ADDR(led_dev.gpio_addr, pin));
	LEDBRD_DBG_PR("pin: %d addr: 0x%p reg: 0x%x\n", pin, GPFSEL_ADDR(led_dev.gpio_addr, pin), reg);

	tmp = GPFSEL_OUTPUT(reg, pin);
	LEDBRD_DBG_PR("tmp: %x\n", tmp);

	iowrite32(GPFSEL_OUTPUT(reg, pin), GPFSEL_ADDR(led_dev.gpio_addr, pin));

#ifdef LEDBRD_DEBUG
	reg = ioread32(GPFSEL_ADDR(led_dev.gpio_addr, pin));
	LEDBRD_DBG_PR("pin: %d addr: 0x%p reg: 0x%x\n", pin, GPFSEL_ADDR(led_dev.gpio_addr, pin), reg);
#endif

	LEDBRD_ROUTINE_EXIT
}

void ledboard_clear_pin(u8 pin)
{

#ifdef LEDBRD_DEBUG
	unsigned int reg_new;
#endif

	LEDBRD_ROUTINE_ENTER

	iowrite32(GPCLR(pin), GPCLR0_ADDR(led_dev.gpio_addr));

#ifdef LEDBRD_DEBUG
	reg_new = ioread32(GPCLR0_ADDR(led_dev.gpio_addr));
	LEDBRD_DBG_PR("pin: %d addr: 0x%p writed: 0x%x new reg: 0x%x\n", pin, GPCLR0_ADDR(led_dev.gpio_addr), GPCLR(pin), reg_new);
#endif

	LEDBRD_ROUTINE_EXIT
}

void ledboard_set_pin_off(u8 pin)
{
	unsigned int reg;

#ifdef LEDBRD_DEBUG
	unsigned int reg_new;
#endif

	LEDBRD_ROUTINE_ENTER

	reg = ioread32(GPFSET0_ADDR(led_dev.gpio_addr));
	LEDBRD_DBG_PR("pin: %d addr: 0x%p reg: 0x%x\n", pin, GPFSET0_ADDR(led_dev.gpio_addr), reg);

	iowrite32(GPFSET_OFF_REG(pin, reg), GPFSET0_ADDR(led_dev.gpio_addr));

#ifdef LEDBRD_DEBUG
	reg_new = ioread32(GPFSET0_ADDR(led_dev.gpio_addr));
	LEDBRD_DBG_PR("pin: %d addr: 0x%p writed: 0x%x new reg: 0x%x\n", pin, GPFSET0_ADDR(led_dev.gpio_addr), GPFSET_OFF_REG(pin, reg), reg_new);
#endif

	LEDBRD_ROUTINE_EXIT
}

void ledboard_set_pin_on(u8 pin)
{
	unsigned int reg;

#ifdef LEDBRD_DEBUG
	unsigned int reg_new;
#endif

	LEDBRD_ROUTINE_ENTER

	reg = ioread32(GPFSET0_ADDR(led_dev.gpio_addr));
	LEDBRD_DBG_PR("pin: %d addr: 0x%p reg: 0x%x\n", pin, GPFSET0_ADDR(led_dev.gpio_addr), reg);
	
	iowrite32(GPFSET_ON_REG(pin, reg), GPFSET0_ADDR(led_dev.gpio_addr));

#ifdef LEDBRD_DEBUG
        reg_new = ioread32(GPFSET0_ADDR(led_dev.gpio_addr));
        LEDBRD_DBG_PR("pin: %d addr: 0x%p writed: 0x%x new reg: 0x%x\n", pin, GPFSET0_ADDR(led_dev.gpio_addr), GPFSET_ON_REG(pin, reg), reg_new);
#endif
	
	LEDBRD_ROUTINE_EXIT
}	

void ledboard_set_pin(u8 pin)
{
#ifdef LEDBRD_DEBUG
	unsigned int reg_new;
#endif

	LEDBRD_ROUTINE_ENTER

	iowrite32(GPFSET_ON(pin), GPFSET0_ADDR(led_dev.gpio_addr));

#ifdef LEDBRD_DEBUG
        reg_new = ioread32(GPFSET0_ADDR(led_dev.gpio_addr));
        LEDBRD_DBG_PR("pin: %d addr: 0x%p writed: 0x%x new reg: 0x%x\n", pin, GPFSET0_ADDR(led_dev.gpio_addr), GPFSET_ON(pin), reg_new);
#endif
	
	LEDBRD_ROUTINE_EXIT
}

module_init(ledboard_init);
module_exit(ledboard_exit);
