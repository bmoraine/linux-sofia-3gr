#ifndef BT_COMPAT
#define BT_COMPAT

static DECLARE_WAIT_QUEUE_HEAD(bt_compat_queue);

#define bt_dev_info(hdev, fmt, ...) \
        BT_INFO("%s: " fmt, (hdev)->name, ##__VA_ARGS__)
#define bt_dev_err(hdev, fmt, ...) \
        BT_ERR("%s: " fmt, (hdev)->name, ##__VA_ARGS__)
#define bt_dev_dbg(hdev, fmt, ...) \
        BT_DBG("%s: " fmt, (hdev)->name, ##__VA_ARGS__)

static inline int wait_on_bit_timeout(void *word, int bit, unsigned mode,
			unsigned long timeout)
{
	int ret;

	might_sleep();
	if (!test_bit(bit, word))
		return 0;

	ret = wait_event_interruptible_timeout(bt_compat_queue,
					       !test_bit(bit, word),
					       timeout);
	if (ret < 1)
		return timeout;
	return 0;
}

static inline struct sk_buff *hci_cmd_sync(struct hci_dev *hdev, u16 opcode,
					   u32 plen, const void *param,
					   u32 timeout)
{
	struct sk_buff *skb;

	if (!test_bit(HCI_UP, &hdev->flags))
		return ERR_PTR(-ENETDOWN);

	bt_dev_dbg(hdev, "opcode 0x%4.4x plen %d", opcode, plen);

	hci_req_lock(hdev);
	skb = __hci_cmd_sync(hdev, opcode, plen, param, timeout);
	hci_req_unlock(hdev);

	return skb;
}

#define wake_up_bit(word, bit) wake_up_interruptible(&bt_compat_queue)

#define smp_mb__after_atomic()

#define HCI_QUIRK_INVALID_BDADDR	15
#define HCI_QUIRK_EXTERNAL_CONFIG	16

#endif /* BT_COMPAT */
