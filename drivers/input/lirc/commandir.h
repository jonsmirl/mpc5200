/*
 *  commandir.h
 */

#define ASCII0      48

/* transmitter channel control */
#define MAX_DEVICES      8
#define MAX_CHANNELS     32
#define TX1_ENABLE       0x80
#define TX2_ENABLE       0x40
#define TX3_ENABLE       0x20
#define TX4_ENABLE       0x10

/* command types */
#define cNothing	0
#define cRESET		1
#define cFLASH		2
#define cLCD		3
#define cRX		4

/* CommandIR control codes */
#define MCU_CTRL_SIZE   3
#define FREQ_HEADER     2
#define RESET_HEADER    3
#define FLASH_HEADER    4
#define LCD_HEADER      5
#define TX_HEADER       7
#define TX_HEADER_NEW   8

/* Queue buffering constants */
#define SEND_IDLE	0
#define SEND_ACTIVE	1

#define QUEUELENGTH	256

extern int cmdir_write(unsigned char *buffer, int count,
		       void *callback_fct, int u);
extern ssize_t cmdir_read(unsigned char *buffer, size_t count);
extern int set_tx_channels(unsigned int next_tx);


static int cmdir_open(struct inode *inode, struct file *file);
static int cmdir_probe(struct usb_interface *interface,
		       const struct usb_device_id *id);
static void cmdir_disconnect(struct usb_interface *interface);
static int cmdir_release(struct inode *inode, struct file *file);
static int cmdir_check(int device_num);
static void init_cmdir_var(int device_num);
static void reset_cmdir(int device_num);
static void update_cmdir_string(int device_num);
static void print_cmdir(int device_num);
static ssize_t cmdir_file_read(struct file *file, char *buffer,
			       size_t count, loff_t *ppos);
ssize_t cmdir_read(unsigned char *buffer, size_t count);
static ssize_t cmdir_file_write(struct file *file, const char *buffer,
				size_t count, loff_t *ppos);
int cmdir_write(unsigned char *buffer, int count, void *callback_fct, int u);
int write_core(unsigned char *buffer, int count,
	       void *callback_fct, int device_num);
static void cmdir_write_bulk_callback(struct urb *urb, struct pt_regs *regs);
int set_tx_channels(unsigned int next_tx);

int add_cmdir_queue(unsigned char *buffer, int count,
		    void *callback_vct, int usecdelay);
int cmdir_write_queue(unsigned char *buffer, int count, void *callback_vct);
int send_queue(void);
int wait_to_tx(int usecs);
