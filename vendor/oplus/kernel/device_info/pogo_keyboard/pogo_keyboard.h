#ifndef __KEYBOARD_CORE__H__
#define __KEYBOARD_CORE__H__

#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
//#include <mt-plat/aee.h>
#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/delay.h>
//#include <mt-plat/mtk_boot_common.h>
#include <linux/hid.h>
#include <linux/hid-debug.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/mod_devicetable.h>

#include <linux/fcntl.h>
#include <linux/sched/signal.h>
#include <linux/sched/task.h>
#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/uio.h>

#include <linux/regulator/consumer.h>


#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/iio/consumer.h>
#include <linux/workqueue.h>
#include <linux/pogo_common.h>

#define WAKEUP_NAME "pogo_wakeup"
#define KEYBOARD_NAME "pogo_keyboard"
#define TOUCHPAD_NAME "pogo_touchpad"
#define KEYBOARD_CORE_NAME "tinno,pogo_keyboard"

#define TOUCH_FINGER_MAX    5

// #define CONFIG_TOUCH_SCREEN //macro to enable touch screen mode for keyboard touchpad, or else mouse mode by default.

#define TOUCH_X_MAX    4096
#define TOUCH_Y_MAX    4096

#define UART_BUFFER_SIZE 256

#define CONFIG_KEYBOARD_DEBUG
#define CONFIG_KEYBOARD_ERR
#define CONFIG_KEYBOARD_INFO


//#define CONFIG_ADC_SUPPORT
//#define CONFIG_PLUG_SUPPORT // use hall sensor for keyboard attachment detection, or else use drx low signal from keyboard.

#define CONFIG_POWER_CTRL_SUPPORT

#define CONFIG_BOARD_V4_SUPPORT

#define CONFIG_KB_DEBUG_FS     // enable keyboard debugging file nodes.

#define KB_TAG   "POGO_KB:"
#ifdef CONFIG_KEYBOARD_INFO
#define kb_info(fmt, args...)   printk(KERN_ERR KB_TAG fmt,##args)
#else
#define kb_info(fmt, args...)   do { } while (0)
#endif


#ifdef CONFIG_KEYBOARD_DEBUG
static bool pogo_debug_en = true;
#else
static bool pogo_debug_en = false;
#endif
#define kb_debug(fmt, args...)  do {\
                                    if(pogo_debug_en)\
                                        printk(KERN_ERR KB_TAG fmt,##args);\
                                } while (0)




#ifdef CONFIG_KEYBOARD_ERR
#define kb_err(fmt, args...)   printk(KERN_ERR KB_TAG fmt,##args)
#else
#define kb_err(fmt, args...)   do { } while (0)
#endif


#define KEYBOARD_CONNECT_STATUS         (1<<0) // keyboard is attached.
#define KEYBOARD_LCD_ON_STATUS          (1<<1) // pad lcd is on.
#define KEYBOARD_CAPSLOCK_ON_STATUS     (1<<2)
#define KEYBOARD_MUTEDISABLE_ON_STATUS  (1<<3)
#define KEYBOARD_MICDISABLE_ON_STATUS   (1<<4)



#define LED_MIC_MUTE            0x0b

#define KEY_LOCKSCREEN          0x280
#define KEY_SWITCHLANUAGE       0x282
#define KEY_MICDISABLE          0x283
#define KEY_TOUCHPANELMUTE      0x284
#define KEY_GLOBALSEARCH        0x285
#define KEY_FULLSCREEN          0x286
#define KEY_SPLITSCREEN         0x287
#define KEY_SUPERINTCON         0x289
#define KEY_CUSTOMERAPP1        0x28a
#define KEY_CUSTOMERAPP2        0x28b
#define KEY_KB_ENABLE           0x28e
#define KEY_KB_DISABLE          0x28f



// CRC types

#define CRC_TYPE_CCITT     0
#define CRC_TYPE_IBM       1

// Polynomial = X^16 + X^12 + X^5 + 1
#define  POLYNOMIAL_CCITT   0x1021

// Polynomial = X^16 + X^15 + X^2 + 1
#define  POLYNOMIAL_IBM    0x8005

//  CRC init value
#define  CRC_CCITT_INIT_VAL   0x1D0F
#define  CRC_IBM_INIT_VAL     0xC596
#define  CRC_IBM_DUNHUANG   0xA5C9
#define CRC_ERROR_VALUE            0xFFFF

// Timer expiry time, unit-ms
#define POWEROFF_TIMER_EXPIRY       50

#define POWEROFF_DISCONNECT_MAX     20
#define POWEROFF_CONNECT_MAX        12
#define POWEROFF_TIMER_CHECK_MAX    20

#define PLUGIN_CHECK_CHECK_MAX      6

#define SYNC_LCD_STATE_CNT_MAX      50

#define KEVENT_LOG_TAG              "psw_bsp_pogopin"
#define KEVENT_EVENT_ID             "pogopin_sn_report"
#define DEFAULT_SN_LEN              24
#define POGOPIN_TRIGGER_MSG_LEN     2048
#define MAX_POGOPIN_EVENT_TAG_LEN   32
#define MAX_POGOPIN_EVENT_ID_LEN    20
#define MAX_POGOPIN_PAYLOAD_LEN     1024

#define DFU_FW_INFO_LEN    64
#define DFU_ONE_WRITY_LEN_MAX    128
#define PROC_PAGE_LEN		50
#define MAX_FW_NAME_LENGTH	60
#define ONE_WRITY_LEN_MAX    52 // 128
#define FW_PROGERSS_0		0
#define FW_PROGERSS_1		1
#define FW_PROGRESS_2		2
#define FW_PROGRESS_3		3
#define FW_PROGRESS_5		5
#define FW_PROGRESS_6		6
#define FW_PROGRESS_20		20
#define FW_PROGRESS_22		22
#define FW_PROGRESS_25		25
#define FW_PROGRESS_45		45
#define FW_PROGRESS_46		46
#define FW_PROGRESS_47		47
#define FW_PROGRESS_48		48
#define FW_PROGRESS_50		50
#define FW_PROGRESS_93		93
#define FW_PROGRESS_96		96
#define FW_PROGRESS_99		99
#define FW_PROGRESS_100        100
#define FW_PERCENTAGE_100      100
#define DEFAULT_KBVER_LEN      13
#define KBVER_LEN_MAX      30//20
#define TPVER_LEN       7
#define KBLOG_LEN_MAX      106

#define UART_PACKET_MIN_HEADER_SIZE      2       // main_cmd + len
#define UART_PACKET_SYNC_HEAD_SIZE       8       // sync start
#define UART_PACKET_SYNC_TAIL_SIZE       4       // sync end
#define UART_PACKET_HEADER_SIZE          16      // 8sync+1start+3adds+2cmd+1len+1
#define UART_PACKET_TAIL_SIZE            4       // (2CRC+1end+4sync)
#define UART_PACKET_CRC_HEADER_SIZE      5

#define UART_PACKET_START_OFFSET         8
#define UART_PACKET_SRC_ADDR_OFFSET      9
#define UART_PACKET_DST_ADDR_OFFSET      10
#define UART_PACKET_MAIN_CMD_OFFSET      11
#define UART_PACKET_LENGTH_OFFSET        12
#define UART_PACKET_DATA_OFFSET          13
//MCU VERSION
#define KBMCU_VESION_1_0_7   0x0107
//disconect count
#define DEFAULT_PLUGIN_DISCONNECT_COUNT     40 // 2s
#define DFU_PLUGIN_DISCONNECT_COUNT     160 //8s
#define DEFAULT_DISCONNECT_COUNT         10      // 0.5s
#define DFU_DISCONNECT_COUNT            40      // 2s
#define DFU_RESET_DISCONNECT_COUNT      400     // 20s
#define TP_OTA_START_DISCONNECT_COUNT   300     // 15s
#define KB_OTA_PROGRESS_INCREMENT   1      // add 1 one time
#define OTA_SLEEP_INTERVAL        50      // 50ms
#define TP_OTA_PROGRESS_INCREMENT   10      // add 10 one time
#define OTA_STATUS_INACTIVE             0
#define OTA_STATUS_ACTIVE               1


enum {
    KEYBOARD_PLUG_IN_EVENT = 0x01,
    KEYBOARD_PLUG_OUT_EVENT,
    KEYBOARD_HOST_LCD_ON_EVENT,
    KEYBOARD_HOST_LCD_OFF_EVENT,

    KEYBOARD_CAPSLOCK_ON_EVENT,
    KEYBOARD_CAPSLOCK_OFF_EVENT,
    KEYBOARD_MUTEDISABLE_ON_EVENT,
    KEYBOARD_MUTEDISABLE_OFF_EVENT,

    KEYBOARD_MICDISABLE_ON_EVENT,
    KEYBOARD_MICDISABLE_OFF_EVENT,
    KEYBOARD_HOST_RX_GPIO_EVENT,
    KEYBOARD_HOST_RX_UART_EVENT,

    KEYBOARD_POWER_ON_EVENT,  // turn on vcc for keyboard
    KEYBOARD_POWER_OFF_EVENT, // turn off vcc for keyboard
    KEYBOARD_HOST_CHECK_EVENT,
    KEYBOARD_TEST_EVENT,
    KEYBOARD_REPORT_SN_EVENT,
    KEYBOARD_REPORT_TOUCH_STATUS_EVENT,
    KEYBOARD_REPORT_KBVER_EVENT,
    KEYBOARD_REPORT_KBLOG_EVENT,
    KEYBOARD_REPORT_NFC_STA,
    POGO_KEYBOARD_EVENT_MAX, //add new event befor it
};

enum {
    KEYBOARD_UART_OPEN_START_TYPE = 0x01,
    KEYBOARD_UART_OPEN_END_TYPE,
    KEYBOARD_UART_WRITE_START_TYPE,
    KEYBOARD_UART_WRITE_END_TYPE,
};

enum {
    KEYBOARD_UART_RX_CLEAR_TYPE = 0x00,
    KEYBOARD_UART_RX_SET_TYPE,
    KEYBOARD_UART_RX_GPIO_TYPE,
};

enum {
    KEYBOARD_UART_RECV_DEFAULT = 0x00,
    KEYBOARD_UART_RECV_START,
    KEYBOARD_UART_RECV_END,
};

enum updateStatus{
    FW_UPDATE_READY = 0,
    FW_UPDATE_START,
    FW_UPDATE_FAIL,
    FW_UPDATE_SUC,
};

struct touch_event {
    unsigned int x;
    unsigned int y;
    unsigned char id;
    unsigned char area;
    unsigned char is_down;
    unsigned char is_left;
    unsigned char is_right;
};

struct pogo_keyboard_event {
    unsigned char event;
};

struct pogo_uevent {
    struct class *uevent_class;
    struct device *uevent_dev;
    char *class_name;
    char *cdev_name;
    char *dev_name;
};

struct pogo_keyboard_data {
    struct input_dev *input_pogo_keyboard;
    unsigned char old[8];
    unsigned char new[8];
    bool is_down; // tracking last/latest key state(true means key is pressed or else released).
    unsigned int down_code; // tracking last/latest key code being pressed.

    /* struct input_dev *input_mm_pogo_keyboard;*/
    unsigned char mm_old[4];
    unsigned char mm_new[4];
    bool is_mmdown; // indicating last/latest multimedia key state(true means key is pressed or else released).
    unsigned int down_mmcode;  // tracking last/latest multimedia key code being pressed.

    bool pogopin_touch_support;
    struct input_dev *input_touchpad;
    unsigned char touchpad_data[5+TOUCH_FINGER_MAX*5];   //len(1Byte)+key(1Byte)+fingers(20Byte). touch packet
    unsigned char  touch_down;
    unsigned char  touch_temp;
    struct touch_event event;
    unsigned int touchpad_x_max;
    unsigned int touchpad_y_max;
    unsigned int touchpad_x_resolution;
    unsigned int touchpad_y_resolution;
    int prev_finger_count;
    bool touchpad_disable_state;
    bool touchpad_gesture_state;
    bool touchpad_gesture_ignore;
    bool keypad_pluginout_state;   //keyboard plugin/out status for factory test.
    struct input_dev *input_wakeup;

    // struct notifier_block pogo_keyboard_notify;
    unsigned char write_buf[UART_BUFFER_SIZE];
    unsigned char read_buf[UART_BUFFER_SIZE];
    unsigned char write_check_buf[UART_BUFFER_SIZE];
    // unsigned char recv_dma_buf[UART_BUFFER_SIZE];
    // int recv_len;
    // int recv_status;

    int read_flag;
    int read_len;

    int write_len;

    struct task_struct *pogo_keyboard_task;
    int flag;

    //int power_status;
    struct platform_device *plat_dev;
    struct pinctrl *pinctrl;
    struct pinctrl_state *uart_tx_set;
    struct pinctrl_state *uart_tx_clear;
    struct pinctrl_state *uart_rx_set;
    struct pinctrl_state *uart_rx_clear;
    struct pinctrl_state *uart_rx_gpio_pin;
    struct pinctrl_state *uart_wake_gpio_pin;
    struct pinctrl_state *uart_wake_clear;
    struct pinctrl_state *pogo_gpio_clear;
    struct pinctrl_state *pogo_power_enable;
    struct pinctrl_state *pogo_power_disable;
    //int status;
    struct regulator *vcc_reg;
    bool get_vcc_from_ldo;
    atomic_t vcc_on;
    int power_en_gpio;
    int plug_gpio;
    int plug_irq;

    int tx_gpio;
    int tx_en_gpio;

    int uart_rx_gpio;
    int uart_rx_gpio_irq;
    int uart_rx_mode;

    int uart_wake_gpio;
    int uart_wake_gpio_irq;

    int uart_tx_gpio;
    struct mutex mutex;
    unsigned char pogo_keyboard_status;
    // unsigned char pogo_keyboard_events;
    struct kfifo event_fifo;
    spinlock_t event_fifo_lock;

    struct file *file_client;
    struct uart_port *port;
    struct hrtimer plug_timer;          // timer for first keyboard attachment detection after system init.
    struct hrtimer heartbeat_timer;         // timer for monitoring keyboard heartbeat report periodically.
    struct hrtimer poweroff_timer;
    struct hrtimer plugin_check_timer;

    int disconnect_count; // numbers of heartbeat packet for plug-out detection after keyboard initialization complete.
    int plug_in_count; // numbers of heartbeat packet for plug-out detection before keyboard initialization complete.
    int poweroff_disconnect_count;
    int poweroff_connect_count;
    int poweroff_timer_check_count;
    int check_connect_count;
    int check_disconnect_count;

    struct drm_panel *active_panel;
    void *notifier_cookie;
#if IS_ENABLED(CONFIG_DEVICE_MODULES_DRM_MEDIATEK) && defined(CONFIG_OPLUS_POGOPIN_FUNCTION)
    struct notifier_block disp_notifier;
#endif

    bool lcd_notify_reg;
    struct delayed_work lcd_notify_reg_work;

    unsigned long long mac_addr;                //keyboard mac, report when plug in by keyboard
    struct pogo_uevent *pogo;
    struct pogo_uevent *nfc;

    char *tty_name;
    char *keyboard_name;
    char *keyboard_ble_name;
    unsigned char keyboard_brand;

    unsigned int sync_lcd_state_cnt;

    bool plug_timer_one_time;
    u8 report_sn[DEFAULT_SN_LEN];
    bool pogo_report_touch_status;

    bool pogo_battery_support;
    u8 pogo_battery_power_level;
    unsigned short crc_ibm_init_val;
    bool get_crc_ibm_from_dts;
    unsigned short pogo_id_product;
    bool is_confidential;

    //for pogopin firmware update
    bool pogopin_fw_support;
    u32 ota_start_addr;
    u32 ota_get_version_addr;
    u32 ota_send_data_start_addr;
    const char *ota_firmware_name;
    struct work_struct  kpdmcu_fw_update_work;
    struct delayed_work kpdmcu_fw_data_version_work;
    struct wakeup_source *pogopin_wakelock;
    int kpdmcu_mcu_version;
    int kpdmcu_fw_data_ver;
    u32 kpdmcu_fw_cnt;
    unsigned char report_kbver[KBVER_LEN_MAX];
    u8 kbver_len;
    u8 kpd_fw_status;
    int fw_update_progress;
    bool kpdmcu_update_end;
    bool kpdmcu_fw_update_force;
    bool is_kpdmcu_need_fw_update;
    bool pogopin_ota_dfu;
    u32 dfu_fwinfo_start_addr;
    unsigned char report_tpver[TPVER_LEN];

    //for trx test
    struct work_struct kpd_trx_test_work;
    u8 kblog_len;
    unsigned char report_kblog[KBLOG_LEN_MAX];
    unsigned char nfc_status;
};

typedef struct {
    uint8_t main_cmd;
    uint8_t sub_cmd;
    uint8_t *ack;
    size_t ack_len;
    void (*handler)(char *buf, struct pogo_keyboard_data *client);
} CommandHandler;

extern struct pogo_keyboard_data *pogo_keyboard_client;
extern char TAG[60];

int pogo_keyboard_write(void *buf, int len);
int pogo_keyboard_read(void *buf, int *r_len);
int pogo_keyboard_write_and_read(void *w_buf, int w_len, void *r_buf, int *r_len);

void pogo_keyboard_show_buf(void *buf, int count);

int pogo_keyboard_input_init(char *keyboard_name);
int pogo_keyboard_input_report(char *buf);
int pogo_keyboard_mm_input_report(char *buf);
int pogo_keyboard_input_wakeup_init(void);

void pogo_keyboard_led_process(int code, int value);
void pogo_keyboard_led_report(int key_value);

int touchpad_input_init(void);
int touchpad_input_report(char *buf);
void reset_tool_buttons(struct input_dev *input_dev);

extern ssize_t pogo_tty_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);

//for ota
extern void kpdmcu_fw_data_version_thread(struct work_struct *work);
extern void kpdmcu_fw_update_thread(struct work_struct *work);
extern int pogo_keyboard_set_lcd_state(bool state);
#endif
