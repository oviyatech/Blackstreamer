#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/ioport.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/unistd.h>
#include <linux/i2c.h>

#define LCD(x) outb((c = ((0x0f & x) | (c & 0xf0) )), 0x378)
#define RS(x) outb((c = (((~(1 << 4)) & c) | (x << 4))), 0x378)
#define EN(x) outb((c = (((~(1 << 5)) & c) | (x << 5))), 0x378)
#define en 0b00010000
#define rs 0b00100000
#define OUT(x) outb(x, 0x378)
#define MAXSIZE 17

static char lcd_buffer1[17];
static unsigned char c;
static char d_buf[MAXSIZE];
static int port;
static char lcd_space = ' ';
static int major = 61;

static int lcd_open(struct inode *inode, struct file *filp);
static int lcd_close(struct inode *inode, struct file *filp);
static ssize_t lcd_read(struct file *filp, char *buf, size_t count, loff_t *f_pos);
static ssize_t lcd_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos);
//static void lcd_strobe(void);
static void data(unsigned char);
static void cmd(unsigned char);
static void clear(void);
//static void i2c_lcd_init(void);
static void printlcd(char *);
#if 0
struct lcd_16x2 {

        struct device *dev;
        int (*write)(struct device *dev, unsigned int reg, unsigned int val);
        int (*read)(struct device *dev, unsigned int reg);


};
#endif

static struct file_operations fops = {
    open: lcd_open,
    read: lcd_read,
    write: lcd_write,
    release: lcd_close
};

static int lcd_open(struct inode *inode, struct file *filp)
{
    printk("OVIYA : LCD OPEN");
    return 0;
}

static int lcd_close(struct inode *inode, struct file *filp)
{
    printk("OVIYA : LCD CLOSE");
    return 0;
}

static ssize_t lcd_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    if(count < MAXSIZE) {
        copy_from_user(d_buf, buf, count);
        d_buf[count] = 0;
        printlcd(d_buf);
        *f_pos += count;
        return count;
        } else {
        copy_from_user(d_buf, buf, MAXSIZE - 1);
        d_buf[MAXSIZE - 1] = 0;
        printlcd(d_buf);
        *f_pos += MAXSIZE - 1;
        return MAXSIZE - 1;
    }
}

static ssize_t lcd_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
     printk("OVIYA : LCD READ");
    return 0;
}

static int lcd_i2c_write(struct device *dev, unsigned int reg,
                                unsigned int val)
{
        struct i2c_client *client = to_i2c_client(dev);
	return 0;
}

static int lcd_i2c_read(struct device *dev, unsigned int reg)
{
         struct i2c_client *client = to_i2c_client(dev);
	return 0;
}

static void lcd_strobe(void)
{
	EN(1);
	udelay(1);
	EN(0);
	udelay(1);
}

static void data(unsigned char data)
{
	RS(1);
	udelay(40);
	LCD(data >> 4);
	lcd_strobe();
	LCD(data);
	lcd_strobe();
	udelay(10);
	RS(0);
	udelay(10);
}

static void cmd(unsigned char command)
{
	RS(0);
	udelay(40);
	LCD(command >>4);
	lcd_strobe();
	LCD(command);
	lcd_strobe();
}

static void clear(void)
{
	cmd(1);
	udelay(2000);
} 

static void printlcd(char *p)
{
	static int count = 0;
	count = 0;
	clear();
	cmd(0x80);
	while(lcd_buffer1[count])
	data(lcd_buffer1[count++]);
	count = 0;
	cmd(0xc0);
	while(*p) {
		if((*p != '\n') && (*p != '\t')) {
			lcd_buffer1[count++] = *p;
			data(*p);
			} else {
			data(lcd_space);
			lcd_buffer1[count++] = lcd_space;
		}
		p++;
	}
	lcd_buffer1[16] = 0;
	msleep(2000);
}

static int __devinit lcd_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
	
	 int a;
    a = register_chrdev(major, "registered_pport_61", &fops);
    if(a < 0) {
        printk(KERN_ALERT "error: can't register major number %d\n", major);
        return a;
    }
    port = check_region(0x378, 1);
    if(port) printk(KERN_ALERT "pport cannot reserve 0x378\n");
    request_region(0x378, 1, "registered_pport_61");
    //lcd setup//
    outb(0, 0x378);
    //udelay(10000);
    msleep(10000);
    //i2c_lcd_init();
    //udelay(10000);
    msleep(10000);
    printlcd("DRIVER INSERTED ");
    return 0;

}


static int __devexit lcd_remove(struct i2c_client *client)
{
	 printk(KERN_ALERT "pport module is going to terminate\n");
    printlcd("DRIVER REMOVED  ");
     unregister_chrdev(major, "registered_pport_61");
//	clear(); 
	return 0;
}


static const struct i2c_device_id lcd_id[] = {
        { "16x2-lcd", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, lcd_id);

static struct i2c_driver lcd_driver = {
        .driver = {
                .name = "16x2_lcd-i2c",
                .owner = THIS_MODULE,
        },
        .probe = lcd_probe,
        .remove = __devexit_p(lcd_remove),
        .id_table = lcd_id,
};

static int __init i2c_lcd_init(void)
{
       return i2c_add_driver(&lcd_driver); /* Oviya commanded*/

}
subsys_initcall(i2c_lcd_init);


static void __exit lcd_exit(void)
{
       i2c_del_driver(&lcd_driver);
}
module_exit(lcd_exit);
MODULE_AUTHOR("oviya technologies");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("LCD I2C driver");
