
struct fm_ld_drv_register {
    /**
    * @priv_data: priv data holder for the FMR driver. This is filled by
    * the FMR driver data during registration, and sent back on
    * ld_drv_reg_complete_cb and cmd_handler members.
    */
void *priv_data;

    /**
    * @ ld_drv_reg_complete_cb: Line Discipline Driver registration complete
    * callback.
    *
    * This function is called by the Line discipline driver when
    * registration is completed.
    *
    * Arg parameter is the fm driver private data
    * data parameter is status of LD registration In
    * progress/Registered/Unregistered.
    */
void (*ld_drv_reg_complete_cb)(void *arg, char data);

    /**
    * @fm_cmd_handler: BT Cmd/Event and IRQ informer
    *
    * This function is called by the line descipline driver, to signal the FM
    * Radio driver on host when FM packet is available. This includes command
    * complete and IRQ forwarding. However the processing of the buff contents,
    * do not happen in the LD driver context.
    *
    * arg parameter is the fm driver private data
    * data parameter is status of LD registration In
    * Progress/Registered/Unregistered.
    */
long (*fm_cmd_handler)(void *arg, struct sk_buff *);

    /**
    * @fm_cmd_write: FM Command Send function
    *
    * This field should be populated by the LD driver. This function is used by
    * FM Radio driver to send the command buffer
    */
long (*fm_cmd_write)(struct sk_buff *skb);
};


extern long register_fmdrv_to_ld_driv(struct fm_ld_drv_register *fm_ld_drv_reg);

extern long unregister_fmdrv_from_ld_driv(
			struct fm_ld_drv_register *fm_ld_drv_reg);


#define LL_SLEEP_ACK    0xF1
#define LL_WAKE_UP_ACK  0xF0

/*---- HCI data types ------------*/
#define HCI_COMMAND_PKT 0x01
#define HCI_ACLDATA_PKT 0x02
#define HCI_SCODATA_PKT 0x03
#define HCI_EVENT_PKT   0x04

/* ---- HCI Packet structures ---- */
#define HCI_COMMAND_HDR_SIZE	3
#define HCI_EVENT_HDR_SIZE	2
#define HCI_ACL_HDR_SIZE	4
#define HCI_SCO_HDR_SIZE	3

#define INVALID -1
#define HCI_COMMAND_COMPLETE_EVENT      0x0E
#define HCI_INTERRUPT_EVENT		0xFF
#define HCI_COMMAND_STATUS_EVT		0x0F
#define FMR_DEBUG_EVENT			0x2B

/*----------- FMR Command ----*/
#define FMR_WRITE       0xFC58
#define FMR_READ	0xFC59
#define FMR_SET_POWER   0xFC5A
#define FMR_SET_AUDIO   0xFC5B
#define FMR_IRQ_CONFIRM 0xFC5C
#define FMR_TOP_WRITE   0xFC5D
#define FMR_TOP_READ    0xFC5E

#define STREAM_TO_UINT16(u16, p) { (p) += 4; u16 = ((unsigned int)(*(p)) + \
					(((unsigned int)(*((p) + 1))) << 8)); }
#define MAX_BT_CHNL_IDS 4

/* ----- HCI receiver states ---- */
#define LBF_W4_H4_HDR   0
#define LBF_W4_PKT_HDR  1
#define LBF_W4_DATA     2

#define N_INTEL_LDISC_BUF_SIZE		4096
#define TTY_THRESHOLD_THROTTLE		128 /* now based on remaining room */
#define TTY_THRESHOLD_UNTHROTTLE	128

#define BT_FW_DOWNLOAD_INIT	_IOR('L', 1, uint64_t)
#define BT_FW_DOWNLOAD_COMPLETE	_IOW('L', 2, uint64_t)

#define RELEVANT_IFLAG(iflag) ((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

#define ENABLE  1
#define DISABLE 0

struct bcm_bt_lpm_platform_data {
	int gpio_wake;          /* CPU -> BCM wakeup gpio */
	int gpio_host_wake;     /* BCM -> CPU wakeup gpio */
	int int_host_wake;      /* BCM -> CPU wakeup irq */
	int gpio_enable;        /* GPIO enable/disable BT/FM */
	int port;               /* UART port to use with BT/FM */
};


enum proto_type {
	LBF_BT,
	LBF_FM = 5,
};

enum ioctl_status {
	DO_FW_DL,
	DO_STACK_INIT,
	FW_FAILED,
	FW_SUCCESS
};


struct st_proto_s {
	enum proto_type type;
	unsigned char chnl_id;
	unsigned char hdr_len;
	unsigned char offset_len_in_hdr;
	unsigned char len_size;
	unsigned char reserve;
};
