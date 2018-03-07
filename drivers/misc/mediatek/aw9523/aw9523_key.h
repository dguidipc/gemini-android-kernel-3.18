#ifndef AW9523B_H
#define AW9523B_H

#ifndef BOOL
    #define BOOL char
#endif

 #ifndef U32
    #define U32 unsigned long
#endif

#ifndef TRUE
    #define TRUE 1
#endif
 
#ifndef FALSE
    #define FALSE 0
#endif
 
#ifndef NULL
    #define NULL ((void*)0)
#endif

#define AW9523_KEY_NAME	"aw9523-key"
 
#define AW9523_TAG                  "[aw9523] "

#define AW9523_ERR(fmt, args...)    printk(KERN_ERR AW9523_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
//#define AW9523_DEBUG
#ifdef AW9523_DEBUG
#define AW9523_FUN(f)    	    printk(KERN_ERR AW9523_TAG"%s\n", __FUNCTION__)
#define AW9523_LOG(fmt, args...)    printk(KERN_ERR AW9523_TAG fmt, ##args)
#else
#define AW9523_LOG(fmt, args...)
#define AW9523_FUN(f)
#endif

#define  AW9523_SCK_PIN     (GPIO136 | 0x80000000)
#define  AW9523_SDA_PIN     (GPIO137 | 0x80000000)
#define  AW9523_RESET_PIN   (GPIO127 | 0x80000000)

//IIC的写地址，={1011，0，AD1，AD0，0}，AD0,AD1接低则为0xB0 ，接高则是0xB6
#define IIC_ADDRESS_WRITE		0xB0       
#define IIC_ADDRESS_READ		0xB1 

#define X_NUM  8   //列
#define Y_NUM  7   //行

//i2c  tranfer ,repeat try times
#define AW9523_I2C_MAX_LOOP  50	

#define AW9523_GPIO_AS_INT(pin)        do{                                                     		\
                                            mt_set_gpio_mode(pin, GPIO_AW9523_EN_PIN_M_EINT);   	\
                                            mt_set_gpio_dir(pin, GPIO_DIR_IN);                  	\
                                            mt_set_gpio_pull_enable(pin, GPIO_PULL_ENABLE);    		\
					    mt_set_gpio_pull_select(pin, GPIO_PULL_UP); 	\
                                        }while(0)

//reg address
#define INPUT_PORT0  	0x00
#define INPUT_PORT1  	0x01
#define OUTPUT_PORT0 	0x02
#define OUTPUT_PORT1 	0x03
#define CONFIG_PORT0 	0x04
#define CONFIG_PORT1 	0x05
#define INT_PORT0 	0x06
#define INT_PORT1 	0x07
#define IC_ID 		0x10
#define CTL 		0x11

typedef struct{
	char name[10];
	int key_code;
	int key_val;
	int row;
	int col;
}KEY_STATE;

#endif
