/*
 ****************************************************************
 *
 *  Component: VLX virtual ethernet driver
 *
 *  Copyright (C) 2012 - 2013 Intel Mobile Communications GmbH
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

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/nk_sofia_bridge.h>
#else
#include <nk/nkern.h>
#endif

/* #define BULLSEYE_HOSTTEST */
/* debugging macros */

#define DBG_ERR(x)	printk(x)
#define VETH_WARN(x...)	printk(x)
#define VETH_INFO(x...)	printk(x)
#define VETH_ERR(x...)	printk(x)
/* #define VETH_DBG(x...)        printk(x) */
#define VETH_DBG(x...)

#define RING_ALIGN(x)  (((x) + (L1_CACHE_BYTES - 1)) & ~(L1_CACHE_BYTES - 1))

/*
 * The communication relies on a data ring with RING_SIZE slots. The ring
 * descriptor and the data slots are all allocated in the same shared memory
 * segment. For performance, it is needed that data copied in the ring are
 * aligned on cache lines. Therefore each slot, ip header and shared info
 * structure are aligned.
*/

typedef struct VEthRingDesc {
	volatile nku32_f p_idx;	/* producer index */
	volatile nku32_f freed_idx;	/* freed slot index */
	volatile nku32_f c_idx;	/* consumer index */
	volatile nku8_f stopped;	/* reflect netdev states (started/stopped) */
	nku16_f size;		/* size of the ring (number of slots) */
} VEthRingDesc;

struct VEthLink;

typedef struct VEthSlotDesc {
	nku32_f len;
	struct VEthLink *link;
#ifndef CONFIG_X86_INTEL_SOFIA
	void *data;
#else
	void *data;		/* virt data ptr for ring receiver */
	void *tx_data;		/* virt data ptr for ring transmitter */
#endif
} VEthSlotDesc;

#ifdef CONFIG_NKERNEL_VETH_RING_SIZE
#define RING_SIZE	    CONFIG_NKERNEL_VETH_RING_SIZE
#else
#define RING_SIZE 	    64
#endif

#if RING_SIZE & (RING_SIZE-1)
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
#define SKB_SHINFO_SIZE     0x200

#define SLOT_HLEN_SIZE      RING_ALIGN(ETH_HLEN)
#define SLOT_HLEN_PAD       (SLOT_HLEN_SIZE - ETH_HLEN)
#define SLOT_DATA_SIZE      RING_ALIGN(ETH_HLEN + ETH_DATA_LEN + SKB_SHINFO_SIZE)
#define DATA_SIZE           (RING_SIZE * (SLOT_HLEN_SIZE + SLOT_DATA_SIZE))

#define SLOT_DESC_SIZE	    RING_ALIGN(sizeof(VEthSlotDesc))
#define DESC_SIZE	    (RING_SIZE * SLOT_DESC_SIZE)

#define RING_DESC_SIZE	    RING_ALIGN(sizeof(VEthRingDesc))

#define PMEM_SIZE	    (RING_DESC_SIZE + DESC_SIZE + DATA_SIZE)

#define RING_P_ROOM(rng)     (RING_SIZE - ((rng)->p_idx - (rng)->freed_idx))
#define RING_IS_FULL(rng)    (((rng)->p_idx - (rng)->freed_idx) >= RING_SIZE)
#define RING_IS_EMPTY(rng)   ((rng)->p_idx == (rng)->freed_idx)

#define RING_C_ROOM(rng)     ((rng)->p_idx - (rng)->c_idx)

/*
 * Helper functions to push/pull data in rx or tx rings.
 */

static inline void ring_push_data(VEthRingDesc *ring, nku8_f *src,
				  unsigned int len)
{
#if defined(BULLSEYE_HOSTTEST)
	return;
#else
	VEthSlotDesc *sd;
	nku8_f *dst;

	int tmp = ring->p_idx - ring->c_idx;

	BUG_ON((tmp < 0) || tmp > RING_SIZE);

	dst = ((nku8_f *) ring) + RING_DESC_SIZE;
	dst += (ring->p_idx & (RING_INDEX_MASK)) * SLOT_DESC_SIZE;

	sd = (VEthSlotDesc *) dst;
	sd->len = len;
#ifdef CONFIG_X86_INTEL_SOFIA
	VETH_DBG("ring_push_data: %08x -> %08x\n",
		 (unsigned int)src, (unsigned int)sd->tx_data);
	memcpy(sd->tx_data + SLOT_HLEN_PAD, src, len);
#else
	VETH_DBG("ring_push_data: %08x -> %08x\n",
		 (unsigned int)src, (unsigned int)sd->data);
	memcpy(sd->data + SLOT_HLEN_PAD, src, len);
#endif

	ring->p_idx++;
#endif
}

static inline unsigned int ring_pull_data(VEthRingDesc *ring, nku8_f *dst)
{
#if defined(BULLSEYE_HOSTTEST)
	return 0;
#else
	VEthSlotDesc *sd;
	nku8_f *src;
	unsigned int len;

	int tmp = ring->p_idx - ring->c_idx;
	BUG_ON((tmp < 0) || tmp > RING_SIZE);

	src = ((nku8_f *) ring) + RING_DESC_SIZE;
	src += (ring->c_idx & (RING_INDEX_MASK)) * SLOT_DESC_SIZE;

	sd = (VEthSlotDesc *) src;
	len = sd->len;
	VETH_DBG("ring_pull_data: %08x <- %08x\n",
		 (unsigned int)dst, (unsigned int)sd->data);
	memcpy(dst, sd->data + SLOT_HLEN_PAD, len);

	ring->c_idx++;
	ring->freed_idx++;

	return len;
#endif
}

/* */

typedef struct {
	NkOsId osid;
	NkXIrq rx_xirq;		/* store rx xirq number */
	NkXIrqId rx_xid;	/* rx xirq handler id */
	NkXIrq tx_ready_xirq;	/* store tx_ready xirq number */
	NkXIrqId tx_ready_xid;	/* tx_ready xirq handler id */
} VEthLocal;

typedef struct {
	NkOsId osid;
	NkXIrq rx_xirq;		/* xirq to send to peer OS */
	NkXIrq tx_ready_xirq;	/* xirq to send to peer OS */
} VEthPeer;

struct VEth;

/*
 * Local data structure for each
 * connection between this OS and
 * a peer OS.
 */
typedef struct VEthLink {
	NkDevVlink *rx_link;	/* RX vlink */
	VEthRingDesc *rx_ring;	/* RX ring */
	nku8_f *rx_data;	/* RX persistent shared memory */

	nku32_f max;
	nku32_f min;
	nku32_f sum;

	NkDevVlink *tx_link;	/* TX vlink */
	VEthRingDesc *tx_ring;	/* TX ring */
	nku8_f *tx_data;	/* TX persistent shared memory */

	struct VEth *veth;
	VEthLocal local;
	VEthPeer peer;

	int enabled;
} VEthLink;

/*
 * Device instance data.
 */
typedef struct VEth {
	struct net_device_stats stats;	/* net statistics     */
	struct net_device *netdev;	/* Linux net device   */

	VEthLink link;		/* link with peer OS data */
} VEth;

#define VETH_MAX	4
static VEth *veth_devices[VETH_MAX];
static unsigned int veth_devices_num;
static NkXIrqId veth_sysconf_id;

#define VETH_PMEM_ID	4
#define VETH_RXIRQ_ID	6
#define VETH_TXIRQ_ID	7

/* */

#ifdef VETH_DEBUG
static void print_link(NkDevVlink *link)
{
	printk(" .s_state=%d, .c_state=%d,", link->s_state, link->c_state);
	printk(" .s_id=%d, .c_id=%d,", link->s_id, link->c_id);
	printk(" .data=%x\n", link->data);
}
#endif

static void veth_module_cleanup(void);

/* */

static struct net_device_stats *veth_get_stats(struct net_device *dev)
{
	VEth *veth = netdev_priv(dev);
	return &(veth->stats);
}

/* TX handler: packets are transmitted through tx_link ring buffer */

static int veth_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
#if defined(BULLSEYE_HOSTTEST)
	return 0;
#else
	VEth *veth = netdev_priv(dev);
	VEthLink *link = &veth->link;
	VEthRingDesc *tx_ring = link->tx_ring;

	/*
	 * Peer OS Link is not ready, set link down
	 * and account error.
	 */
	if (link->tx_link->s_state != NK_DEV_VLINK_ON) {

		VETH_WARN("%s: xmit, peer driver (%d) not ready\n",
			  dev->name, link->peer.osid);
		netif_carrier_off(dev);
		veth->stats.tx_carrier_errors++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/*
	 * Interface is overrunning.
	 */
	if (RING_IS_FULL(tx_ring)) {
		tx_ring->stopped = 1;
		netif_stop_queue(dev);
		veth->stats.tx_fifo_errors++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_BUSY;
	}

	/*
	 * Everything is ok, start xmit.
	 */
	ring_push_data(tx_ring, skb->data, skb->len);

	/*
	 * Statistics
	 */
	veth->stats.tx_bytes += skb->len;
	veth->stats.tx_packets++;
	dev->trans_start = jiffies;

	dev_kfree_skb_any(skb);

	/*
	 * Ring is full, stop interface
	 * and avoid dropping packets
	 */
	if (RING_IS_FULL(tx_ring)) {
		tx_ring->stopped = 1;
		netif_stop_queue(dev);
	}

	nkops.nk_xirq_trigger(link->peer.rx_xirq, link->peer.osid);

	return NETDEV_TX_OK;
#endif
}

/* TX timeout, wake queue up and account error. */

static void veth_tx_timeout(struct net_device *dev)
{
#if defined(BULLSEYE_HOSTTEST)
	return;
#else
	VEth *veth = netdev_priv(dev);
	VEthLink *link = &veth->link;
	VEthRingDesc *tx_ring = link->tx_ring;

	veth->stats.tx_errors++;

	/* If ring is full tell peer OS there is something
	 * to consume. Otherwise, wake up interface.
	 */
	if (RING_IS_FULL(tx_ring)) {
		nkops.nk_xirq_trigger(link->peer.rx_xirq, link->peer.osid);
	} else {
		tx_ring->stopped = 0;
		netif_wake_queue(dev);
	}
#endif
}

#ifdef CONFIG_SKB_DESTRUCTOR

/* FIXME, rephrase the following */
/* This code need some modification in Linux sk_buff management. The files
 * net/core/skbuff.c and include/linux/skbuff.h are modified to provide a
 * destructor handler specific to the data buffer in each skb.
 *
 * Thanks to this feature, it is possible to avoid a second copy when receiving
 * data. Instead, we link the buffer from the shared memory to the skb structure
 * and when the last skb referencing this buffer is freed, we can free the
 * buffer
 */

static void veth_free_buffer(void *data, VEthLink *link)
{
#if defined(BULLSEYE_HOSTTEST)
	return;
#else
	VEthSlotDesc *sd;
	VEthRingDesc *rx_ring = link->rx_ring;
	nku8_f *sptr;
	int was_full = 0;

	VETH_DBG("veth_free_skb %d %d %d\n",
		 rx_ring->p_idx, rx_ring->freed_idx, rx_ring->stopped);

	was_full = RING_IS_FULL(rx_ring);
	sptr = ((nku8_f *) rx_ring) + RING_DESC_SIZE;
	sptr += (rx_ring->freed_idx & (RING_INDEX_MASK)) * SLOT_DESC_SIZE;
	sd = (VEthSlotDesc *) sptr;
	sd->data = data;

	rx_ring->freed_idx++;
	if (rx_ring->stopped && was_full) {
		nkops.nk_xirq_trigger(link->peer.tx_ready_xirq,
				      link->peer.osid);
	}
#endif
}

static void veth_free_skb(struct skb_shared_info *shinfo, void *cookie)
{
#if defined(BULLSEYE_HOSTTEST)
	return;
#endif

	veth_free_buffer(skb_shinfo_to_head(shinfo), cookie);
}

/*
 * Allocate an sk_buff structure for use in rx handler. This
 * function is partly copied from alloc_skb in net/core/skbuff.c
 */

static inline struct sk_buff *veth_alloc_skb(VEthLink *link)
{
	struct skb_shared_info *shinfo;
	struct sk_buff *skb;
	nku32_f size;
	VEthSlotDesc *sd;
	nku8_f *src;
	int tmp;
#if defined(BULLSEYE_HOSTTEST)
	return skb;
#endif
	skb = ___alloc_skb(GFP_ATOMIC, -1);
	if (!skb)
		goto out;

	memset(skb, 0, offsetof(struct sk_buff, tail));
	atomic_set(&skb->users, 1);

	tmp = link->rx_ring->p_idx - link->rx_ring->c_idx;
	BUG_ON((tmp < 0) || tmp > RING_SIZE);

	src = ((nku8_f *) link->rx_ring) + RING_DESC_SIZE;
	src += (link->rx_ring->c_idx & (RING_INDEX_MASK)) * SLOT_DESC_SIZE;
	link->rx_ring->c_idx++;
	sd = (VEthSlotDesc *) src;

	/*
	 * Get size and save link addr and index for later
	 * use in veth_free_skb. No need to check size coherency
	 * as it will later be done in skb_put().
	 */
	size = sd->len;
	sd->link = link;

	skb->len = 0;
	skb->head = sd->data;
	skb->data = sd->data;
	VETH_DBG("ring_pull_ref: %08x\n", (unsigned int)sd->data);
	skb_reset_tail_pointer(skb);
	skb->end = skb->tail + RING_ALIGN(SLOT_HLEN_PAD + size);
	skb->truesize =
	    sizeof(struct sk_buff) + RING_ALIGN(SLOT_HLEN_PAD + size);

	shinfo = skb_shinfo(skb);
	atomic_set(&shinfo->dataref, 1);
	shinfo->nr_frags = 0;
	shinfo->gso_size = 0;
	shinfo->gso_segs = 0;
	shinfo->gso_type = 0;
	shinfo->ip6_frag_id = 0;
	shinfo->frag_list = NULL;
	shinfo->destructor = veth_free_skb;
	shinfo->cookie = link;
	shinfo->orig = NULL;
	shinfo->len = skb_end_pointer(skb) - skb->head;

	skb_reserve(skb, SLOT_HLEN_PAD);
	skb_put(skb, size);

out:
	return skb;
}
#endif

/*
 * Rx xirq handler to receive frames as a rx_ring consummer
 */

static void veth_rx_hdl(void *cookie, NkXIrq xirq)
{
#if defined(BULLSEYE_HOSTTEST)
	return;
#else
	VEthLink *link = (VEthLink *) cookie;
	VEth *veth = link->veth;
	struct net_device *netdev = veth->netdev;	/* Linux net device   */
	VEthRingDesc *rx_ring = link->rx_ring;
	struct sk_buff *skb;
#ifndef CONFIG_SKB_DESTRUCTOR
	nku32_f len;
#endif

	while (RING_C_ROOM(rx_ring) > 0) {
		/*
		 * Check the peer state and account
		 * error if it is not ON.
		 */
		if (link->rx_link->c_state != NK_DEV_VLINK_ON) {
			pr_err("rx_hdl: peer driver not ready\n");
			netif_carrier_off(veth->netdev);
			veth->stats.rx_errors++;
			return;
		}
#ifdef CONFIG_SKB_DESTRUCTOR
		skb = veth_alloc_skb(link);
		if (!skb) {
			nku8_f *src = (nku8_f *) rx_ring + RING_DESC_SIZE;

			src +=
			    (rx_ring->c_idx & (RING_INDEX_MASK)) *
			    SLOT_DESC_SIZE;
			rx_ring->c_idx++;
			veth_free_buffer(src, link);
			veth->stats.rx_dropped++;
			continue;
		}
#else
		/* Over allocate (1514) */
		skb = dev_alloc_skb(ETH_FRAME_LEN);
		if (!skb) {
			rx_ring->c_idx++;
			rx_ring->freed_idx++;
			veth->stats.rx_dropped++;
			continue;
		}
		len = ring_pull_data(rx_ring, skb->data);
		skb_put(skb, len);
#endif
		skb->dev = veth->netdev;
		skb->protocol = eth_type_trans(skb, veth->netdev);
		skb->ip_summed = CHECKSUM_UNNECESSARY;

		/*
		 * Statistics
		 */
		veth->stats.rx_packets++;
		veth->stats.rx_bytes += skb->len;

		netif_rx(skb);
	}
	netdev->last_rx = jiffies;

#ifndef CONFIG_SKB_DESTRUCTOR
	/* Send tx ready xirq if producer ring was stopped (full) */
	if (rx_ring->stopped) {
		nkops.nk_xirq_trigger(link->peer.tx_ready_xirq,
				      link->peer.osid);
	}
#endif
#endif
}

/*
 * Tx_ready xirq handler.
 */

static void veth_tx_ready_hdl(void *cookie, NkXIrq xirq)
{
#if defined(BULLSEYE_HOSTTEST)
	return;
#else
	VEthRingDesc *tx_ring = ((VEthLink *) cookie)->tx_ring;
	VEth *veth = ((VEthLink *) cookie)->veth;

	if (tx_ring->stopped && !RING_IS_FULL(tx_ring)) {
		tx_ring->stopped = 0;
		netif_wake_queue(veth->netdev);
	}
#endif
}

/* vnet_link initialisation is done in 4 steps:

1. find the corresponding vlink (vlink where we are client)
2. allocate a communication ring per vlink
3. allocate and attach irqs
4. send sysconf irq to peer OS to start handshake (handshake is continued in
   sysconf handler)

*/

static void veth_link_reset_rx(VEthLink *link)
{
	VETH_DBG("reset_rx\n");
	link->rx_ring->c_idx = 0;
	link->rx_ring->freed_idx = 0;
}

static void veth_link_reset_tx(VEthLink *link)
{
	VETH_DBG("reset_tx\n");
	link->tx_ring->p_idx = 0;
	link->tx_ring->stopped = 0;
}

/*
 * Send sysconf xirq to peer OS.
 */

static inline void veth_sysconf_trigger(NkOsId osid)
{
	nkops.nk_xirq_trigger(NK_XIRQ_SYSCONF, osid);
}

/*
 * Handshake function to update link states.
 */
static int veth_handshake_rx(VEthLink *link)
{
	volatile int *my_state;
	volatile int peer_state;
	int need_sysconf = 0;

	VETH_DBG("> rx_handshake\n\t%d->%d\n",
		 link->rx_link->c_state, link->rx_link->s_state);

	my_state = (volatile int *)&link->rx_link->s_state;
	peer_state = (volatile int)link->rx_link->c_state;

	switch (*my_state) {
	case NK_DEV_VLINK_OFF:
		if (peer_state != NK_DEV_VLINK_ON) {
			veth_link_reset_rx(link);
			*my_state = NK_DEV_VLINK_RESET;
			need_sysconf = 1;
		}
		break;
	case NK_DEV_VLINK_RESET:
		if (peer_state != NK_DEV_VLINK_OFF) {
			*my_state = NK_DEV_VLINK_ON;
			need_sysconf = 1;
		}
		break;
	case NK_DEV_VLINK_ON:
		if (peer_state == NK_DEV_VLINK_OFF) {
			veth_link_reset_rx(link);
			*my_state = NK_DEV_VLINK_RESET;
			need_sysconf = 1;
		}
		break;
	}

	VETH_DBG("\t%d->%d\n", link->rx_link->c_state, link->rx_link->s_state);

	return need_sysconf;
}

static int veth_handshake_tx(VEthLink *link)
{
	volatile int *my_state;
	volatile int peer_state;
	int need_sysconf = 0;

	VETH_DBG("> tx_handshake\n\t%d->%d\n",
		 link->tx_link->c_state, link->tx_link->s_state);

	my_state = (volatile int *)&link->tx_link->c_state;
	peer_state = (volatile int)link->tx_link->s_state;

	switch (*my_state) {
	case NK_DEV_VLINK_OFF:
		if (peer_state != NK_DEV_VLINK_ON) {
			veth_link_reset_tx(link);
			*my_state = NK_DEV_VLINK_RESET;
			need_sysconf = 1;
		}
		break;
	case NK_DEV_VLINK_RESET:
		if (peer_state != NK_DEV_VLINK_OFF) {
			*my_state = NK_DEV_VLINK_ON;
			need_sysconf = 1;
		}
		break;
	case NK_DEV_VLINK_ON:
		if (peer_state == NK_DEV_VLINK_OFF) {
			veth_link_reset_tx(link);
			*my_state = NK_DEV_VLINK_RESET;
			need_sysconf = 1;
		}
		break;
	}

	VETH_DBG("\t%d->%d\n", link->tx_link->c_state, link->tx_link->s_state);

	return need_sysconf;
}

/*
 * Sysconf xirq handler. This function triggers handshakes for each links.
 */

static void veth_sysconf_hdl(void *cookie, NkXIrq xirq)
{
	int i;

	/* start handshake or change server states according to peer states */
	for (i = 0; i < veth_devices_num; i++) {
		VEthLink *link = &(veth_devices[i]->link);
		int changed = 0;

		if (link->enabled) {
			changed = veth_handshake_rx(link);
			changed |= veth_handshake_tx(link);

			if (changed)
				veth_sysconf_trigger(link->peer.osid);

			if ((link->rx_link->c_state == NK_DEV_VLINK_ON) &&
			    (link->rx_link->s_state == NK_DEV_VLINK_ON) &&
			    (link->tx_link->c_state == NK_DEV_VLINK_ON) &&
			    (link->tx_link->s_state == NK_DEV_VLINK_ON)) {

				if (!netif_carrier_ok(veth_devices[i]->netdev)) {
					printk
					    ("%s: link on (OS#%d <-> OS#%d).\n",
					     veth_devices[i]->netdev->name,
					     link->local.osid, link->peer.osid);
					netif_carrier_on(veth_devices[i]->
							 netdev);
				}
			} else {
				if (netif_carrier_ok(veth_devices[i]->netdev)) {
					printk
					 ("%s: link off (OS#%d <-> OS#%d).\n",
					     veth_devices[i]->netdev->name,
					     link->local.osid, link->peer.osid);
					netif_carrier_off(veth_devices[i]->
							  netdev);
				}
			}
		}
	}
}

static inline void rx_ring_data_init(VEthRingDesc *ring)
{
	VEthSlotDesc *sd;
	nku8_f *sptr;
	nku8_f *dptr;
	int i;

	sptr = ((nku8_f *) ring) + RING_DESC_SIZE;
	dptr = sptr + DESC_SIZE;
	for (i = 0; i < RING_SIZE; i++) {
		sd = (VEthSlotDesc *) sptr;
		sd->data = dptr;
		sptr += SLOT_DESC_SIZE;
		dptr += SLOT_DATA_SIZE;
	}
}

#ifdef CONFIG_X86_INTEL_SOFIA
static inline void tx_ring_tx_data_init(VEthRingDesc *ring)
{
	VEthSlotDesc *sd;
	nku8_f *sptr;
	nku8_f *dptr;
	int i;

	sptr = ((nku8_f *) ring) + RING_DESC_SIZE;
	dptr = sptr + DESC_SIZE;
	for (i = 0; i < RING_SIZE; i++) {
		sd = (VEthSlotDesc *) sptr;
		sd->tx_data = dptr;
		sptr += SLOT_DESC_SIZE;
		dptr += SLOT_DATA_SIZE;
	}
}
#endif

    /* Allocate communication rings (ring, shared memory, xirq).
     * Communication rings may have already been allocated by the peer OS,
     * but this is managed transparently by nkddi.
     */

static int veth_alloc_link_resources(VEthLink *link)
{
	NkDevVlink *rx_link = link->rx_link;
	NkDevVlink *tx_link = link->tx_link;
	VEthRingDesc *rx_ring;
	VEthRingDesc *tx_ring;
	NkPhAddr paddr;

	/*
	 * Allocate RX vlink resources
	 */
	/* Allocate persistent shared memory */
	paddr = nkops.nk_pmem_alloc(nkops.nk_vtop(rx_link), VETH_PMEM_ID,
				    PMEM_SIZE);
	if (paddr == 0) {
		VETH_WARN("OS#%d->OS#%d link=%d server pmem alloc failed.\n",
			  rx_link->c_id, rx_link->s_id, rx_link->link);
		return -ENOMEM;
	}

	rx_ring = (VEthRingDesc *) nkops.nk_mem_map(paddr, PMEM_SIZE);
	if (rx_ring == 0) {
		pr_err("error while mapping\n");
		return -ENOMEM;
	}
	rx_ring->p_idx = 0;
	rx_ring_data_init(rx_ring);

	link->rx_data = ((nku8_f *) rx_ring) + RING_DESC_SIZE;

	link->rx_ring = rx_ring;
	link->local.osid = rx_link->s_id;

	/* Allocate local rx_hdl xirq */
	link->local.rx_xirq = nkops.nk_pxirq_alloc(nkops.nk_vtop(rx_link),
						   VETH_RXIRQ_ID,
						   link->local.osid, 1);
	if (link->local.rx_xirq == 0) {
		VETH_WARN("OS#%d->OS#%d link=%d server pxirq alloc failed.\n",
			  rx_link->c_id, rx_link->s_id, rx_link->link);
		return -ENOMEM;
	}

	/* Attach local rx_hdl xirq handler */
	link->local.rx_xid = nkops.nk_xirq_attach(link->local.rx_xirq,
						  veth_rx_hdl, link);

	if (link->local.rx_xid == 0) {
		VETH_WARN
		  ("OS#%d->OS#%d link=%d server cannot attach xirq handler.\n",
		     rx_link->c_id, rx_link->s_id, rx_link->link);
		return -ENOMEM;
	}

	/* Allocate local tx_ready_hdl xirq handler */
	link->local.tx_ready_xirq = nkops.nk_pxirq_alloc(nkops.nk_vtop(tx_link),
							 VETH_TXIRQ_ID,
							 link->local.osid, 1);
	if (link->local.tx_ready_xirq == 0) {
		VETH_WARN("OS#%d->OS#%d link=%d server pxirq alloc failed.\n",
			  rx_link->c_id, rx_link->s_id, rx_link->link);
		return -ENOMEM;
	}

	/* Attach local tx_ready_hdl xirq handler */
	link->local.tx_ready_xid =
	    nkops.nk_xirq_attach(link->local.tx_ready_xirq, veth_tx_ready_hdl,
				 link);

	if (link->local.tx_ready_xid == 0) {
		VETH_WARN
		  ("OS#%d->OS#%d link=%d server cannot attach xirq handler.\n",
		     rx_link->c_id, rx_link->s_id, rx_link->link);
		return -ENOMEM;
	}

	/*
	 * Allocate TX vlink resources
	 */
	/* Allocate persistent shared memory */
	paddr = nkops.nk_pmem_alloc(nkops.nk_vtop(tx_link), VETH_PMEM_ID,
				    PMEM_SIZE);
	if (paddr == 0) {
		VETH_WARN("OS#%d->OS#%d link=%d client pmem alloc failed.\n",
			  tx_link->c_id, tx_link->s_id, tx_link->link);
		return -ENOMEM;
	}

	tx_ring = (VEthRingDesc *) nkops.nk_mem_map(paddr, PMEM_SIZE);

	if (tx_ring == 0) {
		pr_err("error while mapping\n");
		return -ENOMEM;
	}

	tx_ring->c_idx = 0;
	tx_ring->freed_idx = 0;
#ifdef CONFIG_X86_INTEL_SOFIA
	tx_ring_tx_data_init(tx_ring);
#endif

	link->tx_data = ((nku8_f *) tx_ring) + RING_DESC_SIZE;

	link->tx_ring = tx_ring;
	link->peer.osid = tx_link->s_id;

	/* Allocate peer rx_hdl xirq */
	link->peer.rx_xirq = nkops.nk_pxirq_alloc(nkops.nk_vtop(tx_link),
						  VETH_RXIRQ_ID,
						  link->peer.osid, 1);
	if (link->peer.rx_xirq == 0) {
		VETH_WARN("OS#%d->OS#%d link=%d client pxirq alloc failed.\n",
			  tx_link->c_id, tx_link->s_id, tx_link->link);
		return -ENOMEM;
	}

	/* Allocate peer tx_ready_hdl xirq handler */
	link->peer.tx_ready_xirq = nkops.nk_pxirq_alloc(nkops.nk_vtop(rx_link),
							VETH_TXIRQ_ID,
							link->peer.osid, 1);
	if (link->peer.tx_ready_xirq == 0) {
		VETH_WARN("OS#%d->OS#%d link=%d client pxirq alloc failed.\n",
			  rx_link->c_id, rx_link->s_id, rx_link->link);
		return -ENOMEM;
	}

	return 0;
}

/*
 * Open interface
 */
static int veth_open(struct net_device *dev)
{
#if defined(BULLSEYE_HOSTTEST)
	return 0;
#else

	netif_start_queue(dev);

	return 0;
#endif
}

/*
 * Close interface
 */
static int veth_close(struct net_device *dev)
{
#if defined(BULLSEYE_HOSTTEST)
	return 0;
#else
	netif_stop_queue(dev);

	return 0;
#endif
}

/*
 * Parse mac address in VLX command line:
 *
 * vdev=(veth,linkid|xx:xx:xx:xx:xx:xx)
 *
 */
static void veth_parse_mac_address(VEth *vethdev, NkDevVlink *vlink)
{
	char tmp_addr[6];
	char *opt;
	char *end;
	int i;

	/*
	 * Set mac address to default 00:00:00:00:link:osid
	 */
	vethdev->netdev->dev_addr[4] = vlink->link;
	vethdev->netdev->dev_addr[5] = nkops.nk_id_get();

	/*
	 * Parse vlink s_info field to find
	 * mac address.
	 */
	if (vlink->s_info == 0)
		return;

	opt = (char *)nkops.nk_ptov((NkPhAddr) vlink->s_info);

	for (i = 0; i < 5; i++) {
		tmp_addr[i] = simple_strtoul(opt, &end, 16);
		if ((end == opt) || (*end != ':')) {
			VETH_ERR("error while parsing %s mac address.\n",
				 vethdev->netdev->name);
			return;
		}
		opt = end + 1;	/* skip colon */
	}
	tmp_addr[i] = simple_strtoul(opt, &end, 16);
	if (end == opt) {
		VETH_ERR("error while parsing %s mac address.\n",
			 vethdev->netdev->name);
		return;
	}

	VETH_INFO("%s: setting mac address to %02x:%02x:%02x:%02x:%02x:%02x.\n",
		  vethdev->netdev->name,
		  tmp_addr[0], tmp_addr[1], tmp_addr[2],
		  tmp_addr[3], tmp_addr[4], tmp_addr[5]);

	for (i = 0; i < 6; i++)
		vethdev->netdev->dev_addr[i] = tmp_addr[i];
}

#if defined(BULLSEYE_HOSTTEST)
extern void hosttest_add_test(char *test_name, void (*test_func) (void));
void hosttest_func_veth(void);
#endif

static int
veth_dev_init(VEth *vethdev, NkDevVlink *rx_link, NkDevVlink *tx_link)
{
	int res;

	veth_parse_mac_address(vethdev, rx_link);

	vethdev->link.rx_link = rx_link;
	vethdev->link.tx_link = tx_link;

	res = veth_alloc_link_resources(&vethdev->link);
	if (res < 0) {
		pr_err("%s: error while allocating link resources.\n",
		       vethdev->netdev->name);
		return res;
	};

	/*
	 * This device has all resources allocated.
	 */
	veth_devices[veth_devices_num] = vethdev;
	veth_devices_num++;
	vethdev->link.enabled = 1;
	veth_sysconf_hdl(0, 0);
#if defined(BULLSEYE_HOSTTEST)
	hosttest_add_test("veth", hosttest_func_veth);
#endif
	return 0;
}

#ifndef SET_MODULE_OWNER
#define SET_MODULE_OWNER(dev) do { } while (0)
#endif

static const struct net_device_ops veth_netdev_ops = {
	.ndo_open = veth_open,
	.ndo_stop = veth_close,
	.ndo_start_xmit = veth_start_xmit,
	.ndo_get_stats = veth_get_stats,
	.ndo_tx_timeout = veth_tx_timeout
};

/*
 * Allocate VEth and linux netdev structures.
 */
static VEth *veth_dev_alloc(void)
{
	struct net_device *netdev;
	VEth *veth;
	int res;

	if (veth_devices_num >= VETH_MAX)
		return NULL;

	netdev = alloc_netdev(sizeof(VEth), "veth%d", ether_setup);
	if (netdev == NULL) {
		VETH_ERR("VETH: alloc_etherdev() failed.\n");
		return NULL;
	}

	veth = netdev_priv(netdev);

	/* Reset stats */
	memset(&veth->stats, 0, sizeof(veth->stats));

	veth->netdev = netdev;
	veth->link.veth = veth;
	veth->link.enabled = 0;
	veth->link.local.osid = nkops.nk_id_get();

	SET_MODULE_OWNER(netdev);
	netdev->netdev_ops = &veth_netdev_ops;
	netdev->watchdog_timeo = 3 * HZ;
	netdev->irq = 0;
	netdev->dma = 0;

	/* register new Ethernet interface */
	res = register_netdev(netdev);
	if (res) {
		VETH_WARN("%s: register_netdev() failed (%d)\n", netdev->name,
			  res);
		free_netdev(netdev);
		return NULL;
	}

	/* set link as disconnected */
	netif_carrier_off(netdev);

	return veth;
}

static void veth_dev_free(VEth *veth)
{
#if defined(BULLSEYE_HOSTTEST)
	return;
#else
	if (veth == NULL)
		return;
#endif
	unregister_netdev(veth->netdev);
	free_netdev(veth->netdev);
}

static void veth_dev_create(NkDevVlink *rx_link, NkDevVlink *tx_link)
{
	VEth *veth;

	veth = veth_dev_alloc();

	if (veth == NULL) {
		pr_err("Could not allocate new veth device.\n");
		veth_dev_free(veth);
		return;
	}

	veth_dev_init(veth, rx_link, tx_link);
}

static NkDevVlink *veth_find_pair_vlink(NkDevVlink *l)
{
	NkPhAddr plink;
	NkDevVlink *vlink;

	plink = 0;

	while ((plink = nkops.nk_vlink_lookup("veth", plink))) {
		vlink = nkops.nk_ptov(plink);

		if ((vlink != l) &&
		    (vlink->s_id == l->c_id) &&
		    (vlink->c_id == l->s_id) && (vlink->link == l->link)) {
			/*
			 * We found the vlink linking the other way.
			 */
			return vlink;
		}
	}

	return NULL;
}

static int veth_vlink_in_use(NkDevVlink *vlink)
{
	int i;

	for (i = 0; i < veth_devices_num; i++) {
		if (veth_devices[i]->link.rx_link->link == vlink->link)
			return 1;
	}
	return 0;
}

static int __init veth_module_init(void)
{
	NkPhAddr plink;
	NkDevVlink *vlink;
	NkDevVlink *rx_link;
	NkDevVlink *tx_link;
	int myid;

	/* TODO: add ident */
	pr_info("veth: VLX virtual Ethernet device driver 1.2.\n");

	/*
	 * Check the skb shared info.
	 */
	if (sizeof(struct skb_shared_info) > SKB_SHINFO_SIZE) {
		pr_err("veth: SKB_SHINFO_SIZE (%x) less than real size (%x)\n",
		       SKB_SHINFO_SIZE, sizeof(struct skb_shared_info));
		return -ENOMEM;
	}
	/*
	 * Attach sysconfig handler.
	 */
	veth_sysconf_id =
	    nkops.nk_xirq_attach(NK_XIRQ_SYSCONF, veth_sysconf_hdl, 0);
	if (veth_sysconf_id == 0) {
		pr_err("veth: cannot attach xirq\n");
		veth_module_cleanup();
		return -ENOMEM;
	}

	/*
	 * Probe bidirectionnal vlinks and create veth devices.
	 */
	plink = 0;
	rx_link = NULL;
	tx_link = NULL;
	myid = nkops.nk_id_get();

	while ((plink = nkops.nk_vlink_lookup("veth", plink))) {
		vlink = nkops.nk_ptov(plink);
		pr_info("veth: id:%d / vlink id:%d\n", myid, vlink->s_id);

		if (vlink->s_id == myid && !veth_vlink_in_use(vlink)) {
			/* We found the first vlink (rx_link), now
			 * find the vlink linking the other way.
			 */
			pr_info("veth: RX link found\n");
			rx_link = vlink;
			tx_link = veth_find_pair_vlink(rx_link);
			if (tx_link) {
				pr_info("veth: TX link found\n");
				/* We found a bidirectionnal link, now
				 * we can setup a veth device.
				 */
				veth_dev_create(rx_link, tx_link);
			}
		}
	}
	pr_info("VLX virtual Ethernet device driver initialized\n");

	return 0;
}

/*
 * Cleanup virtual Ethernet device driver
 */

static void veth_module_cleanup(void)
{
	int i;
#if defined(BULLSEYE_HOSTTEST)
	return;
#endif
	if (veth_sysconf_id != 0)
		nkops.nk_xirq_detach(veth_sysconf_id);

	for (i = 0; i < veth_devices_num; i++) {
		if (veth_devices[i]->link.enabled) {
			veth_devices[i]->link.rx_link->s_state =
			    NK_DEV_VLINK_OFF;
			veth_devices[i]->link.tx_link->c_state =
			    NK_DEV_VLINK_OFF;
			nkops.nk_xirq_detach(veth_devices[i]->link.local.
					     rx_xid);
			nkops.nk_xirq_detach(veth_devices[i]->link.local.
					     tx_ready_xid);
			veth_sysconf_trigger(veth_devices[i]->link.peer.osid);
			veth_dev_free(veth_devices[i]);
		}
	}
	veth_devices_num = 0;
}

#if defined(BULLSEYE_HOSTTEST)
static void veth_module_exit(void)
{
	return;
}
#else
static void __exit veth_module_exit(void)
{
	veth_module_cleanup();
}
#endif

#if defined(BULLSEYE_HOSTTEST)
void hosttest_func_veth(void)
{
	VEthRingDesc *test_ring;
	nku8_f *test_src, test_dst;
	unsigned int test_len;
	struct sk_buff *test_skb;
	struct net_device *test_dev;
	void *test_data;
	VEthLink *test_link;
	struct skb_shared_info *test_shinfo;
	void *test_cookie;
	NkXIrq test_xirq;
	VEth *test_veth;

	ring_push_data(test_ring, test_src, test_len);
	ring_pull_data(test_ring, test_dst);
	veth_start_xmit(test_skb, test_dev);
	veth_tx_timeout(test_dev);
	veth_free_buffer(test_data, test_link);
	veth_alloc_skb(test_link);
	veth_free_skb(test_shinfo, test_cookie);
	veth_rx_hdl(test_cookie, test_xirq);
	veth_tx_ready_hdl(test_cookie, test_xirq);
	veth_open(test_dev);
	veth_close(test_dev);
	veth_dev_free(test_veth);
	veth_module_cleanup();
	veth_module_exit();
}
#endif

#ifdef MODULE
MODULE_DESCRIPTION("VLX virtual Ethernet device driver");
MODULE_AUTHOR("Christophe Augier <christophe.augier@redbend.com>");
MODULE_LICENSE("GPL");
#endif

module_init(veth_module_init);
#ifndef BULLSEYE_HOSTTEST
module_exit(veth_module_exit);
#endif
