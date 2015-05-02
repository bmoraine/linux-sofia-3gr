/*
 ****************************************************************
 *
 *  Component: MobileVisor virtual ethernet driver
 *
 *  Copyright (C) 2012 - 2014 Intel Mobile Communications GmbH
 *  Copyright (C) 2011, Red Bend Ltd.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *  Contributor(s):
 *    Christophe Augier (christophe.augier@redbend.com)
 *    Pascal Piovesan (pascal.piovesan@redbend.com)
 *    Adam Mirowski (adam.mirowski@redbend.com)
 *
 ****************************************************************
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/atomic.h>
#include <linux/bitops.h>

#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/if_ether.h>
#include <linux/rtnetlink.h>
#include <linux/skbuff.h>

#include <sofia/mv_gal.h>
#include <sofia/mv_ipc.h>
#include <sofia/mv_hypercalls.h>

#define DEBUG_LEVEL 1
#define DEBUG_LEVEL_ERROR 1
#define DEBUG_LEVEL_INFO  2
#define MODULE_NAME "mveth"

#define mveth_info(fmt, arg...) \
	do { \
		if (DEBUG_LEVEL >= DEBUG_LEVEL_INFO) \
			pr_info(MODULE_NAME"[I]: "fmt, ##arg); \
	} while (0)

#define mveth_error(fmt, arg...) \
	do { \
		if (DEBUG_LEVEL >= DEBUG_LEVEL_ERROR) \
			pr_info(MODULE_NAME"[E]: "fmt, ##arg); \
	} while (0)

#define RING_ALIGN(x)  (((x) + (L1_CACHE_BYTES - 1)) & ~(L1_CACHE_BYTES - 1))

#ifdef CONFIG_NKERNEL_VETH_RING_SIZE
#define RING_SIZE CONFIG_NKERNEL_VETH_RING_SIZE
#else
#define RING_SIZE 64
#endif

#if RING_SIZE & (RING_SIZE - 1)
#error RING_SIZE is not a power of 2
#endif

#define RING_INDEX_MASK	    (RING_SIZE - 1)

/*
 * The skb_shared_info structure is located at the end of the skb data.
 * The size of this structure may vary, depending on the Linux version.
 * Because the SLOT_SIZE must always be the same, we cannot use
 * sizeof, and thefore a fixed value is given and a test is done
 * to verify that this value is great enough.
 */
#define SKB_SHINFO_SIZE 0x200

#define SLOT_HLEN_SIZE RING_ALIGN(ETH_HLEN)
#define SLOT_HLEN_PAD  (SLOT_HLEN_SIZE - ETH_HLEN)
#define SLOT_DATA_SIZE RING_ALIGN(ETH_HLEN + ETH_DATA_LEN + SKB_SHINFO_SIZE)

/*ethernet_header
*/
#define ETH_ALEN            6
#define ETH_DST_OFFSET      0
#define ETH_SRC_OFFSET      ETH_ALEN

struct ethernet_header {
	char dst_addr[ETH_ALEN];
	char src_addr[ETH_ALEN];
	unsigned char type[2];
};

enum mbox_status {
	MBOX_DISCONNECTED,
	MBOX_CONNECTED
};

enum mveth_status {
	MVETH_CLOSED,
	MVETH_OPENED,
};

enum mveth_event_id {
	ETH_EVENT = 0,
	RING0_EVENT,		/* write event */
	RING1_EVENT		/* read event */
};

enum mveth_ring_status {
	RING_START = 0,
	RING_STOP = 1
};

/*
 * The communication relies on a data ring with RING_SIZE slots. The ring
 * descriptor and the data slots are all allocated in the same shared memory
 * segment. For performance, it is needed that data copied in the ring are
 * aligned on cache lines. Therefore each slot, ip header and shared info
 * structure are aligned.
*/
struct mveth_ringslot {
	uint32_t slot_len;	/* only ip packets are put into mailbox */
	uint8_t slot_data[SLOT_DATA_SIZE];
};

struct mveth_ring {
	volatile uint32_t reader_index;
	volatile uint32_t writer_index;
	volatile uint32_t freed_index;
	volatile enum mveth_ring_status status;
	volatile uint32_t ring_event;
	volatile uint32_t size;
	struct mveth_ringslot buf[RING_SIZE];
};

/* shared data to show the local and peer status */
struct mveth_event {
	volatile uint32_t handshake;
	volatile enum mveth_status mveth_status[2];
	uint32_t reserved;
};

struct mveth_instance {
	char name[MAX_MBOX_INSTANCE_NAME_SIZE];
	uint32_t token;

	struct net_device_stats stats;	/* Linux net statistics     */
	struct net_device *netdev;	/* Linux net device   */

	struct ethernet_header eth_header;
	uint32_t open_count;
	uint32_t writer_id;

	struct semaphore read_sem;
	struct semaphore write_sem;

	/* will add read and write blocking if kernel thread is used later
	   wait_queue_t should be here */

	enum mbox_status mbox_status;
	uint32_t eth_event;

	/* Share mem struct */
	struct mveth_event *p_event;
	struct mveth_ring *ring[2];
};

#define RING_DESC_SIZE	    RING_ALIGN(sizeof(struct mveth_ring))

#define RING_P_ROOM(rng)     (RING_SIZE - ((rng)->writer_index - \
				(rng)->freed_index))

#define RING_IS_FULL(rng)    (((rng)->writer_index - (rng)->freed_index) \
				>= RING_SIZE)

#define RING_IS_EMPTY(rng)   ((rng)->writer_index == (rng)->freed_index)
#define RING_C_ROOM(rng)     ((rng)->writer_index - (rng)->reader_index)

#define set_local_status(dev, s) (dev->p_event->mveth_status[dev->writer_id] \
					= s)

#define get_local_status(dev)    (dev->p_event->mveth_status[dev->writer_id])
#define get_peer_status(dev)    (dev->p_event->mveth_status[!dev->writer_id])

#define is_ring0_writer(dev) (dev->writer_id == 0)

#ifdef SAVE_ETH_HEADER
unsigned int first_packet = 0;
/*
 * return value shows if this packet is a ip packet (both ipv4 and ipv6) or not.
*/
static int mveth_save_header(struct mveth_instance *dev, const char *buffer,
			     const unsigned len)
{
	const char *typeLen = buffer + 2 * ETH_ALEN;

	if (len < ETH_HLEN)
		return 0;
	/* this is faked MAC address reading from cmdline in mvconfig */
	if (first_packet == 0) {
		memcpy(dev->eth_header.src_addr, buffer + ETH_DST_OFFSET,
		       ETH_ALEN);
		memcpy(dev->eth_header.dst_addr, buffer + ETH_SRC_OFFSET,
		       ETH_ALEN);
		first_packet = 1;
	}

	/* only ip packet can be put into ring buffer,
	   others are filtered out */
	if ((((unsigned char)typeLen[0] == 0x08)
	     && ((unsigned char)typeLen[1] == 0x00))
	    || (((unsigned char)typeLen[0] == 0x86)
		&& ((unsigned char)typeLen[1] == 0xdd)))
		return 1;
	return 0;
}
#endif

/*
 * Helper functions to push/pull ip packet into/from rx or tx rings.
 * Ring slot only contains ip packet without ethernet header.
 * So length must be larger than 40 bytes (>=40B),
 * this is minimus ip packet size(ip header+TCP/UDP header)
 */
static inline void ring_push_data(struct mveth_ring *ring, char *src,
				  unsigned int len)
{
	struct mveth_ringslot *slot;

	int tmp = ring->writer_index - ring->reader_index;

	BUG_ON((tmp < 0) || tmp > RING_SIZE);	/* ring buffer is corrupted. */

	slot = &ring->buf[ring->writer_index & RING_INDEX_MASK];

	/* we need trim ethernet header off. */
	slot->slot_len = len;
	memcpy(slot->slot_data, src, slot->slot_len);

	ring->writer_index++;
}

static inline unsigned int ring_pull_data(struct mveth_ring *ring, char *dst)
{
	struct mveth_ringslot *slot;

	int tmp = ring->writer_index - ring->reader_index;
	BUG_ON((tmp < 0) || tmp > RING_SIZE);

	slot = &ring->buf[ring->reader_index & RING_INDEX_MASK];

	/* only ip packet is pulled out */
	memcpy(dst, slot->slot_data, slot->slot_len);

	ring->reader_index++;
	ring->freed_index++;

	return slot->slot_len;
}

#define MVETH_MAX	2
static struct mveth_instance *mveths[MVETH_MAX];
static unsigned int mveth_count;

static struct net_device_stats *mveth_get_stats(struct net_device *dev)
{
	struct mveth_instance *mveth = netdev_priv(dev);
	return &(mveth->stats);
}

/* TX handler: packets are transmitted through tx_link ring buffer */
/* skb has the ethernet head here. */
static int mveth_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct mveth_instance *mveth = netdev_priv(dev);
	struct mveth_ring *wr_ring = mveth->ring[mveth->writer_id];

	/*
	 * Peer OS Link is not ready, set link down
	 * and account error.
	 */
	if (get_peer_status(mveth) != MVETH_OPENED) {
		netif_carrier_off(dev);
		mveth->stats.tx_carrier_errors++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* We drop the non-ip packet here */
#ifdef SAVE_ETH_HEADER
	if (mveth_save_header(mveth, skb->data, skb->len) == 0) {
		mveth->stats.tx_errors++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
#endif

	/*
	 * Interface is overrunning.
	 */
	if (RING_IS_FULL(wr_ring)) {
		wr_ring->status = RING_STOP;
		mv_ipc_mbox_post(mveth->token, wr_ring->ring_event);
		mveth->stats.tx_fifo_errors++;
		netif_stop_queue(dev);
		dev_kfree_skb_any(skb);
		return NETDEV_TX_BUSY;
	}

	wr_ring->status = RING_START;
	/*
	 * Everything is ok, start xmit without ethernet header.
	 */
	ring_push_data(wr_ring, skb->data + ETH_HLEN, skb->len - ETH_HLEN);

	/*
	 * Statistics (without ethernet_header)
	 */
	mveth->stats.tx_bytes += skb->len - ETH_HLEN;
	mveth->stats.tx_packets++;
	dev->trans_start = jiffies;

	dev_kfree_skb_any(skb);

	if (RING_IS_FULL(wr_ring)) {
		wr_ring->status = RING_STOP;
		netif_stop_queue(dev);
	}

	mv_ipc_mbox_post(mveth->token, wr_ring->ring_event);
	return NETDEV_TX_OK;
}

/* TX timeout, wake queue up and account error. */
static void mveth_tx_timeout(struct net_device *dev)
{
	struct mveth_instance *mveth = netdev_priv(dev);
	struct mveth_ring *wr_ring = mveth->ring[mveth->writer_id];

	mveth->stats.tx_errors++;
	/* If ring is full tell peer OS there is something
	 * to consume. Otherwise, wake up interface.
	 */
	if (RING_IS_FULL(wr_ring))
		mv_ipc_mbox_post(mveth->token, wr_ring->ring_event);
	else {
		wr_ring->status = RING_START;
		netif_wake_queue(dev);
	}
}

/*
 * Rx xirq handler to receive frames as a rx_ring consummer
 * when we get a hirq from peer
 */
static irqreturn_t mveth_rx_hdl(struct mveth_instance *mveth)
{
	struct net_device *netdev = mveth->netdev;
	struct mveth_ring *rd_ring = mveth->ring[!mveth->writer_id];
	struct sk_buff *skb;
#ifdef CONFIG_SKB_DESTRUCTOR
	struct mveth_ringslot *slot;
#endif
	unsigned int len;

	while (RING_C_ROOM(rd_ring) > 0) {
		/*
		 * Check the peer state and account
		 * error if it is not ON.
		 */
		if (get_peer_status(mveth) != MVETH_OPENED) {
			pr_err("rx_hdl: peer driver not ready\n");
			netif_carrier_off(netdev);
			mveth->stats.rx_errors++;
			return IRQ_HANDLED;
		}

#ifdef CONFIG_SKB_DESTRUCTOR
		slot = &rd_ring->buf[rd_ring->reader_index & RING_INDEX_MASK];
		skb = build_skb(slot->slot_data, sizeof(slot->slot_data));
		if (unlikely(!skb)) {
			rd_ring->reader_index++;
			rd_ring->freed_index++;
			mveth->stats.rx_dropped++;
			continue;
		}
		rd_ring->reader_index++;
		rd_ring->freed_index++;
		skb_put(skb, slot->slot_len);
#else
		/* Over allocate (1500) */
		skb = dev_alloc_skb(ETH_FRAME_LEN - ETH_HLEN);
		if (!skb) {
			rd_ring->reader_index++;
			rd_ring->freed_index++;
			mveth->stats.rx_dropped++;
			continue;
		}
		len = ring_pull_data(rd_ring, skb->data);
		skb_put(skb, len);
#endif
		skb->dev = netdev;
		if ((skb->data[0] & 0xF0) == 0x40)
			skb->protocol = htons(ETH_P_IP);
		else
			skb->protocol = htons(ETH_P_IPV6);
		skb->ip_summed = CHECKSUM_UNNECESSARY;
		skb_reset_mac_header(skb);

		/*
		 * Statistics
		 */
		mveth->stats.rx_packets++;
		mveth->stats.rx_bytes += skb->len;

		netif_rx(skb);
	}
	netdev->last_rx = jiffies;

	/* Send tx ready xirq if producer ring was stopped (full) */
	if (rd_ring->status == RING_STOP)
		mv_ipc_mbox_post(mveth->token, rd_ring->ring_event);

	return IRQ_HANDLED;
}

/*
 * Tx_ready xirq handler.
 */
static irqreturn_t mveth_tx_ready_hdl(struct mveth_instance *mveth)
{
	struct mveth_ring *wr_ring = mveth->ring[mveth->writer_id];

	if (wr_ring->status == RING_STOP && !RING_IS_FULL(wr_ring)) {
		wr_ring->status = RING_START;
		netif_wake_queue(mveth->netdev);
	}
	return IRQ_HANDLED;
}

static void mveth_link_reset_rx(struct mveth_instance *mveth)
{
	struct mveth_ring *rd_ring = mveth->ring[!mveth->writer_id];
	rd_ring->reader_index = 0;
	rd_ring->freed_index = 0;
	rd_ring->status = RING_START;
}

static void mveth_link_reset_tx(struct mveth_instance *mveth)
{
	struct mveth_ring *wr_ring = mveth->ring[mveth->writer_id];
	wr_ring->writer_index = 0;
	wr_ring->status = RING_START;
}

/*
 * Open interface and handshake
 */
static int mveth_open(struct net_device *dev)
{
	netif_start_queue(dev);
	return 0;
}

/*
 * Close interface
 */
void mveth_close(struct mveth_instance *dev)
{
	dev->open_count = 0;

	/* reset ring */
	dev->ring[dev->writer_id]->writer_index = 0;
	dev->ring[dev->writer_id]->status = RING_START;

	dev->ring[!dev->writer_id]->reader_index = 0;
	dev->ring[!dev->writer_id]->freed_index = 0;
	dev->ring[!dev->writer_id]->status = RING_START;

	/* wake up read and write */
	netif_wake_queue(dev->netdev);
}

static int mveth_release(struct net_device *dev)
{
	netif_stop_queue(dev);
	return 0;
}

static void mveth_on_connect(uint32_t token, void *cookie)
{
	struct mveth_instance *dev = (struct mveth_instance *)cookie;

	mveth_info("on connect event!\n");

	if (dev->mbox_status == MBOX_CONNECTED) {
		mveth_error
		("Received on connect irq, while dev is still connected!\n");
		return;
	}

	/* Setup rings according to handshake */
	mveth_info("Setup rings\n");

	if (dev->p_event->handshake == mv_gal_os_id()) {
		dev->writer_id = 0;
		dev->ring[dev->writer_id]->ring_event = RING0_EVENT;
	} else {
		dev->writer_id = 1;
		dev->ring[dev->writer_id]->ring_event = RING1_EVENT;
	}

	mveth_link_reset_rx(dev);
	mveth_link_reset_tx(dev);

	set_local_status(dev, MVETH_CLOSED);
	dev->mbox_status = MBOX_CONNECTED;

}

static void mveth_on_disconnect(uint32_t token, void *cookie)
{
	struct mveth_instance *dev = (struct mveth_instance *)cookie;
	mveth_info("on disconnect event!\n");

	if (dev->mbox_status == MBOX_DISCONNECTED) {
		mveth_error("Received disconnect,but dev is disconnected!\n");
		return;
	}
	mveth_link_reset_rx(dev);
	mveth_link_reset_tx(dev);

	set_local_status(dev, MVETH_CLOSED);

	if (netif_carrier_ok(dev->netdev))
		netif_carrier_off(dev->netdev);

	dev->mbox_status = MBOX_DISCONNECTED;
}

static void mveth_on_event(uint32_t token, uint32_t event_id, void *cookie)
{
	struct mveth_instance *dev = (struct mveth_instance *)cookie;
	switch (event_id) {
	case RING0_EVENT:
		mveth_info("on ring0 event!\n");
		if (is_ring0_writer(dev))
			mveth_tx_ready_hdl(dev);
		else
			mveth_rx_hdl(dev);
		break;
	case RING1_EVENT:
		mveth_info("on ring1 event!\n");
		if (is_ring0_writer(dev))
			mveth_rx_hdl(dev);
		else
			mveth_tx_ready_hdl(dev);
		break;
	case ETH_EVENT:
		mveth_info("on eth event!\n");
		switch (get_peer_status(dev)) {
		case MVETH_OPENED:
			dev->open_count++;
			set_local_status(dev, MVETH_OPENED);
			mveth_link_reset_rx(dev);
			mveth_link_reset_tx(dev);
			if (!netif_carrier_ok(dev->netdev))
				netif_carrier_on(dev->netdev);
			break;
		case MVETH_CLOSED:
			dev->open_count--;
			set_local_status(dev, MVETH_CLOSED);
			mveth_close(dev);
			if (netif_carrier_ok(dev->netdev))
				netif_carrier_off(dev->netdev);
			break;
		}
		break;
	default:
		break;
	}
}

/*
 * Parse mac address in mbox command line:
 *
 * vdev=(veth,linkid|xx:xx:xx:xx:xx:xx)
 *
 */
static void mveth_parse_mac_address(struct mveth_instance *mveth, char *cmdline)
{
	unsigned long tmp_addr[6];
	char *opt;
	char *end;
	int i;

	/*
	 * Set mac address to default 00:00:00:00:writerid:osid
	 */
	mveth->netdev->dev_addr[4] = mveth->writer_id;
	mveth->netdev->dev_addr[5] = mv_gal_os_id();

	/*
	 * Parse cmdline field to find
	 * mac address.
	 */
	if ((cmdline == NULL) || (*cmdline == 0))
		return;

	i = 0;
	opt = cmdline;
	while (*opt != 0) {
		end = opt;
		while (*end != ':' && *end != 0)
			end++;
		*end = 0;
		if (kstrtoul(opt, 16, &tmp_addr[i++])) {
			mveth_error("error while parsing %s mac address.\n",
				    mveth->netdev->name);
			return;
		}
		if (i >= 4)
			break;
		opt = end + 1;
	}

	for (i = 0; i < 4; i++)
		mveth->netdev->dev_addr[i] = tmp_addr[i];
}

#ifndef SET_MODULE_OWNER
#define SET_MODULE_OWNER(dev) do { } while (0)
#endif

static const struct net_device_ops mveth_netdev_ops = {
	.ndo_open = mveth_open,
	.ndo_stop = mveth_release,
	.ndo_start_xmit = mveth_start_xmit,
	.ndo_get_stats = mveth_get_stats,
	.ndo_tx_timeout = mveth_tx_timeout
};

static void mveth_dev_free(struct mveth_instance *mveth)
{
	if (mveth == NULL)
		return;
	unregister_netdev(mveth->netdev);
	free_netdev(mveth->netdev);
}

static struct mbox_ops mveth_mbox_ops = {
	.on_connect = mveth_on_connect,
	.on_disconnect = mveth_on_disconnect,
	.on_event = mveth_on_event
};

void on_mveth_instance(char *instance_name, uint32_t instance_index,
		       uint32_t instance_count)
{
	unsigned char *shared_mem_start;
	unsigned int shared_mem_size;
	char *cmdline;
	struct mveth_instance *mveth;
	uint32_t token;
	void *p_share_mem;
	int res;
	struct net_device *netdev;

	mveth_count = instance_count;

	/* allocate net_device for each instance */
	netdev = alloc_netdev(sizeof(struct mveth_instance),
			"veth%d", ether_setup);
	if (netdev == NULL) {
		mveth_error("VETH: alloc_etherdev() failed.\n");
		return;
	}

	mveth = netdev_priv(netdev);
	mveth->netdev = netdev;	/* link them together */
	mveths[instance_index] = mveth;	/* store the instance pointer */

	strncpy(mveth->name, instance_name, MAX_MBOX_INSTANCE_NAME_SIZE);

	/* Reset stats */
	memset(&mveth->stats, 0, sizeof(mveth->stats));
	mveth->eth_event = ETH_EVENT;
	mveth->mbox_status = MBOX_DISCONNECTED;
	mveth->writer_id = 0;
	mveth->open_count = 0;

	SET_MODULE_OWNER(netdev);
	netdev->netdev_ops = &mveth_netdev_ops;
	netdev->watchdog_timeo = 3 * HZ;
	netdev->irq = 0;
	netdev->dma = 0;

	/* register new Ethernet interface */
	res = register_netdev(netdev);
	if (res) {
		mveth_error("VETH: register network deivce failed.\n");
		free_netdev(netdev);
		return;
	}

	/* set link as disconnected */
	netif_carrier_off(netdev);

	pr_info("mveth enumerate instance name %s\n", instance_name);
	token = mv_ipc_mbox_get_info("mveth", instance_name, &mveth_mbox_ops,
				     &shared_mem_start, &shared_mem_size,
				     &cmdline, mveth);
	if (token != MBOX_INIT_ERR) {
		mveth->token = token;
		/*
		 * |------------------| <- share mem start / mveth_event start
		 * | mveth event     |
		 * |------------------| <- ring 0 start
		 * | ring 0 structure |
		 * |------------------| <- ring 0 buf start
		 * | ring 0 buf       |  <-- mveth->ring_size
		 * |------------------| <- ring 1 start
		 * | ring 1 structure |
		 * |------------------| <- ring 1 buf start
		 * | ring 1 buf       |  <-- mveth->ring_size
		 * |------------------| <- share mem end
		 */

		/* p_share_mem = share mem start */
		p_share_mem = shared_mem_start;
		mveth->p_event = (struct mveth_event *)p_share_mem;
		if (mveth->p_event == NULL)
			panic("failed to init mveth event");

		/* p_share_mem = ring 0 start */
		p_share_mem += sizeof(struct mveth_event);
		mveth->ring[0] = (struct mveth_ring *)p_share_mem;
		mveth->ring[0]->size = sizeof(mveth->ring[0]->buf);
		/* p_share_mem = ring 1 start */
		p_share_mem += sizeof(struct mveth_ring);
		mveth->ring[1] = (struct mveth_ring *)p_share_mem;
		mveth->ring[1]->size = sizeof(mveth->ring[1]->buf);

		sema_init(&mveth->read_sem, 1);
		sema_init(&mveth->write_sem, 1);

		mveth_parse_mac_address(mveth, cmdline);
		mveth->p_event->handshake = mv_gal_os_id();
		mv_mbox_set_online(mveth->token);
	} else
		panic("failed to init instance\n");
}

static int __init mveth_module_init(void)
{
	pr_info("mveth: virtual Ethernet device driver 1.0.\n");
	mv_ipc_mbox_for_all_instances("mveth", on_mveth_instance);
	pr_info("virtual Ethernet device driver initialized\n");
	return 0;
}

/*
 * Cleanup virtual Ethernet device driver
 */
static void mveth_module_cleanup(void)
{
	int i;
	for (i = 0; i < mveth_count; i++) {

		mv_mbox_set_offline(mveths[i]->token);
		mveth_dev_free(mveths[i]);
	}
	mveth_count = 0;
}

static void __exit mveth_module_exit(void)
{
	mveth_module_cleanup();
}

#ifdef MODULE
MODULE_DESCRIPTION("MobileVisor virtual Ethernet device driver");
MODULE_AUTHOR("Christophe Augier<christophe.augier@redbend.com>");
MODULE_LICENSE("GPL");
#endif

module_init(mveth_module_init);
module_exit(mveth_module_exit);
