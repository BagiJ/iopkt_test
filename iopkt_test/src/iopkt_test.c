//-ptcpip -d/data/home/qnxuser/libiopkt_test.so
/*
 * Copyright (c) 2007, 2014, 2015 QNX Software Systems. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
//#include <sys/types_bsd.h>
//#include <net/if.h>
#include <io-pkt/iopkt_driver.h>
#include <sys/io-pkt.h>
#include <sys/sockio.h>
#include <sys/syspage.h>
#include <sys/device.h>
#include <device_qnx.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/netbyte.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#if 0
#include <net80211/ieee80211_var.h>
#endif

int sam_entry(void *dll_hdl, struct _iopkt_self *iopkt, char *options);

int sam_init(struct ifnet *);
void sam_stop(struct ifnet *, int);

void sam_start(struct ifnet *);
int sam_ioctl(struct ifnet *, unsigned long, caddr_t);

const struct sigevent * sam_isr(void *, int);
int sam_process_interrupt(void *, struct nw_work_thread *);
int sam_enable_interrupt(void *);

void sam_shutdown(void *);


struct _iopkt_drvr_entry IOPKT_DRVR_ENTRY_SYM(sam) = IOPKT_DRVR_ENTRY_SYM_INIT(sam_entry);

#ifdef VARIANT_a
#include <nw_dl.h>
/* This is what gets specified in the stack's dl.c */
struct nw_dll_syms sam_syms[] = {
        {"iopkt_drvr_entry", &IOPKT_DRVR_ENTRY_SYM(sam)},
        {NULL, NULL}
};
#endif

const uint8_t etherbroadcastaddr[ETHER_ADDR_LEN] =
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

struct sam_dev {
	struct device		sc_dev;	/* common device */
#if 1
	struct ethercom		sc_ec;	/* common ethernet */
#else
	struct ieee80211com	sc_ic;	/* common 80211 */
#endif
	nic_config_t		cfg;	/* nic information */
	/* whatever else you need follows */
	struct _iopkt_self	*sc_iopkt;
	int			sc_iid;
	int			sc_irq;
	int			sc_intr_cnt;
	int			sc_intr_spurious;
	struct _iopkt_inter	sc_inter;
	void			*sc_sdhook;
};

int sam_attach(struct device *, struct device *, void *);
int sam_detach(struct device *, int);

CFATTACH_DECL(sam,
	sizeof(struct sam_dev),
	NULL,
	sam_attach,
	sam_detach,
	NULL);

/*
 * Initial driver entry point.
 */
int
sam_entry(void *dll_hdl,  struct _iopkt_self *iopkt, char *options)
{
	printf("In sam_entry\n");
	int		instance, single;
	struct device	*dev;
	void		*attach_args;

	/* parse options */

	/* do options imply single? */
	single = 1;

	/* initialize to whatever you want to pass to sam_attach() */
	attach_args = NULL;

	for (instance = 0;;) {
		/* Apply detection criteria */

		/* Found one */
		dev = NULL; /* No Parent */
		if (dev_attach("sam", options, &sam_ca, attach_args,
		    &single, &dev, NULL) != EOK) {
			break;
		}
		dev->dv_dll_hdl = dll_hdl;
		instance++;


		if (/* done_detection || */ single)
			break;
	}

	if (instance > 0)
		return EOK;

	return ENODEV;
}

int
sam_attach(struct device *parent, struct device *self, void *aux)
{
	printf("In sam_attach\n");
	int			err;
	struct sam_dev		*sam;
	struct ifnet		*ifp;
	uint8_t			enaddr[ETHER_ADDR_LEN];
	struct qtime_entry	*qtp;

	/* initialization and attach */

	sam = (struct sam_dev *)self;
	ifp = &sam->sc_ec.ec_if;

	sam->sc_iopkt = iopkt_selfp;

	/*
	 * CAUTION: As an example we attach to the system timer interrupt.
	 * This would be the network hardware interrupt in a real
	 * driver. When this sample driver is run it masks and unmasks
	 * the system timer interrupt in io-pkt. This may cause problems
	 * with other timer calls in other drivers, potentially even
	 * leading to a deadlock. It is safe to run by itself in io-pkt.
	 */
	qtp = SYSPAGE_ENTRY(qtime);
	sam->sc_irq = qtp->intr;

	if ((err = interrupt_entry_init(&sam->sc_inter, 0, NULL,
	    IRUPT_PRIO_DEFAULT)) != EOK)
		return err;

	sam->sc_inter.func   = sam_process_interrupt;
	sam->sc_inter.enable = sam_enable_interrupt;
	sam->sc_inter.arg    = sam;

	sam->sc_iid = -1; /* not attached yet */

	/* set capabilities */
#if 1
	ifp->if_capabilities_rx = IFCAP_CSUM_IPv4 | IFCAP_CSUM_TCPv4 | IFCAP_CSUM_UDPv4;
	ifp->if_capabilities_tx = IFCAP_CSUM_IPv4 | IFCAP_CSUM_TCPv4 | IFCAP_CSUM_UDPv4;

	sam->sc_ec.ec_capabilities |= ETHERCAP_JUMBO_MTU;
#endif

	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;

	/* Set callouts */
	ifp->if_ioctl = sam_ioctl;
	ifp->if_start = sam_start;
	ifp->if_init = sam_init;
	ifp->if_stop = sam_stop;
	IFQ_SET_READY(&ifp->if_snd);

	ifp->if_softc = sam;

	/* More callouts for 80211... */

	strcpy(ifp->if_xname, sam->sc_dev.dv_xname);
	if_attach(ifp);

	{
		int i;
		for (i = 0; i < ETHER_ADDR_LEN; i++)
			enaddr[i] = i;
	}
#if 1
	/* Normal ethernet */
	ether_ifattach(ifp, enaddr);
#else
	/* 80211 */
	memcpy(sam->sc_ic.ic_myaddr, enaddr, ETHER_ADDR_LEN);
	ieee80211_ifattach(&sam->sc_ic);
#endif
	sam->sc_sdhook = shutdownhook_establish(sam_shutdown, sam);

	return EOK;
}

void sam_set_multicast(struct sam_dev *sam)
{
	printf("In sam_set_multicast\n");
	struct ethercom			*ec = &sam->sc_ec;
	struct ifnet			*ifp = &ec->ec_if;
	struct ether_multi		*enm;
	struct ether_multistep		step;

	ifp->if_flags &= ~IFF_ALLMULTI;

	ETHER_FIRST_MULTI(step, ec, enm);
	while (enm != NULL) {
                if (memcmp(enm->enm_addrlo, enm->enm_addrhi, ETHER_ADDR_LEN)) {
                        /*
                         * We must listen to a range of multicast addresses.
                         * For now, just accept all multicasts, rather than
                         * trying to filter out the range.
                         * At this time, the only use of address ranges is
                         * for IP multicast routing.
                         */
                        ifp->if_flags |= IFF_ALLMULTI;
			break;
                }
		/* Single address */
		printf("Add %2x:%2x:%2x:%2x:%2x:%2x to mcast filter\n",
		       enm->enm_addrlo[0],enm->enm_addrlo[1],
		       enm->enm_addrlo[0],enm->enm_addrlo[1],
		       enm->enm_addrlo[0],enm->enm_addrlo[1]);

	   ETHER_NEXT_MULTI(step, enm);
	}

	if ((ifp->if_flags & IFF_ALLMULTI) != 0) {
		printf("Enable multicast promiscuous\n");
	} else {
		printf("Disable multicast promiscuous\n");
	}
}

int
sam_init(struct ifnet *ifp)
{
	printf("In sam_init\n");
	int		ret;
	struct sam_dev	*sam;

	/*
	 * - enable hardware.
	 *   - look at ifp->if_capenable_[rx/tx]
	 *   - enable promiscuous / multicast filter.
	 * - attach to interrupt.
	 */

	sam = ifp->if_softc;

	if(memcmp(sam->cfg.current_address, LLADDR(ifp->if_sadl), ifp->if_addrlen)) {
		memcpy(sam->cfg.current_address, LLADDR(ifp->if_sadl), ifp->if_addrlen);
		/* update the hardware */
	}
	sam->cfg.verbose = 5;

/*	if (sam->sc_iid == -1) {
		if ((ret = InterruptAttach_r(sam->sc_irq, sam_isr,
		    sam, sizeof(*sam), _NTO_INTR_FLAGS_TRK_MSK)) < 0) {
			printf("InterruptAttach_r failed; exiting init\n");
			return -ret;
		}
		sam->sc_iid = ret;
	}*/
	sam->sc_iid = 50;
	printf("sam_init set if_flag == IFF_RUNNING\n");

	sam_set_multicast(sam);
	ifp->if_flags |= IFF_RUNNING;
	if_link_state_change(ifp, LINK_STATE_UP);

	return EOK;
}

void
sam_stop(struct ifnet *ifp, int disable)
{
	printf("In sam_stop\n");
	struct sam_dev	*sam;

	/*
	 * - Cancel any pending io
	 * - Clear any interrupt source registers
	 * - Clear any interrupt pending registers
	 * - Release any queued transmit buffers.
	 */

	sam = ifp->if_softc;

	if (disable) {
		if (sam->sc_iid != -1) {
			InterruptDetach(sam->sc_iid);
			sam->sc_iid = -1;
		}
		/* rxdrain */
	}

	ifp->if_flags &= ~IFF_RUNNING;
}

/*on TX the if_start() driver callback obtains packets to transmit from the ifp->if_snd queue*/
void
sam_start(struct ifnet *ifp)
{
	printf("In sam_start\n");
	struct sam_dev		*sam;
	struct mbuf		*m;
	struct nw_work_thread	*wtp;
	int  size_written;
	char data_buff[MSIZE];

	sam = ifp->if_softc;
	wtp = WTP;
	int fd;

	fd = open( "/data/file.dat",
				O_RDWR | O_APPEND | O_CREAT, S_IRUSR | S_IWUSR);

	for (;;) {
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;

		/*
		 * Can look at m to see if you have the resources
		 * to transmit it.
		 */

		IFQ_DEQUEUE(&ifp->if_snd, m);

		/* process file */
		m_copydata(m, 0, m->m_hdr.mh_len, data_buff);
/*		int i = 0;
		for (i=0; i < m->m_hdr.mh_len; i++)
		{
			printf("%x ", data_buff[i] & 0xff);
		}*/

		/*strcpy(data, m->M_dat->MH_databuf);*/

		/* You're now committed to transmitting it */

		if( fd != -1 ) {
		    /* write the text              */
		    size_written = write( fd, data_buff,
		    		m->m_hdr.mh_len );
		    printf("Size written in file: %d\n; mh_len %d\n", size_written, m->m_hdr.mh_len);

		    /* test for error              */
		    if( size_written != m->m_hdr.mh_len  ) {
		        perror( "Error writing myfile.dat" );
		        return /*EXIT_FAILURE*/;
		    }
		}

		if (sam->cfg.verbose) {
			printf("Packet sent\n");
		}
		m_freem(m);

		ifp->if_opackets++;  // for ifconfig -v
		// or if error:  ifp->if_oerrors++;
	}
	if( fd != -1 ) {
		printf("closing file");
        close( fd );
	}

	NW_SIGUNLOCK_P(&ifp->if_snd_ex, iopkt_selfp, wtp);
}

int
sam_ioctl(struct ifnet *ifp, unsigned long cmd, caddr_t data)
{
	printf("In sam_ioctl\n");
	struct sam_dev	*sam;
	int		error;

	sam = ifp->if_softc;
	error = 0;

	switch (cmd) {
	case SIOCAIFADDR:
		printf("SIOCAIFADDR option");
		break;
	default:
		error = ether_ioctl(ifp, cmd, data);
		if (error == ENETRESET) {
			/*
			 * Multicast list has changed; set the
			 * hardware filter accordingly.
			 */
			if ((ifp->if_flags & IFF_RUNNING) == 0) {
				/*
				 * Interface is currently down: sam_init()
				 * will call sam_set_multicast() so
				 * nothing to do
				 */
			} else {
				/*
				 * interface is up, recalculate and
				 * reprogram the hardware.
				 */
				sam_set_multicast(sam);
			}
			error = 0;
		}
		break;
	}

	return error;
}

int
sam_detach(struct device *dev, int flags)
{
	printf("In sam_detach\n");
	struct sam_dev	*sam;
	struct ifnet	*ifp;

	/*
	 * Clean up everything.
	 *
	 * The interface is going away but io-pkt is staying up.
	 */
	sam = (struct sam_dev *)dev;
	ifp = &sam->sc_ec.ec_if;

	sam_stop(ifp, 1);
#if 1
	ether_ifdetach(ifp);
#else
	ieee80211_ifdetach(&sam->sc_ic);
#endif

	if_detach(ifp);

	shutdownhook_disestablish(sam->sc_sdhook);

	return EOK;
}

void
sam_shutdown(void *arg)
{
	struct sam_dev	*sam;

	/* All of io-pkt is going away.  Just quiet hardware. */

	sam = arg;

	sam_stop(&sam->sc_ec.ec_if, 1);
}

#ifndef HW_MASK
const struct sigevent *
sam_isr(void *arg, int iid)
{
	printf("In sam_isr\n");
	struct sam_dev		*sam;
	struct _iopkt_inter	*ient;

	sam = arg;
	ient = &sam->sc_inter;

	/*
	 * Close window where this is referenced in sam_enable_interrupt().
	 * We may get an interrupt, return a sigevent and have another
	 * thread start processing on SMP before the InterruptAttach()
	 * has returned.
	 */
	sam->sc_iid = iid;

	InterruptMask(sam->sc_irq, iid);

	return interrupt_queue(sam->sc_iopkt, ient);
}
#else
const struct sigevent *
sam_isr(void *arg, int iid)
{
	struct sam_dev		*sam;
	struct _iopkt_self	*iopkt;
	const struct sigevent	*evp;
	struct inter_thread	*itp;

	sam = arg;
	iopkt = sam->sc_iopkt;
	evp = NULL;

#ifdef READ_CAUSE_IN_ISR
	/*
	 * Trade offs.
	 * - Doing this here means another register read across the bus.
	 * - If not sharing interrupts, this boils down to exactly the
	 *   same amount of work but doing more of it in the isr.
	 * - If sharing interupts, can short circuit some work in the
	 *   stack here.
	 * - Maybe trade off is to only do it if we're detecting
	 *   spurious interrupts which should happen under heavy
	 *   shared interrupt load?
	 */
#ifdef READ_CAUSE_ONLY_ON_SPURIOUS
	if (ient->spurrious) {
#endif
		if (ient->on_list == 0 &&
		    (sam->sc_intr_cause = i82544->reg[I82544_ICR]) == 0) {
			return NULL; /* Not ours */
		}
		sam->sc_flag |= CAUSE_VALID;
#ifdef READ_CAUSE_ONLY_ON_SPURIOUS
	}
#endif
#endif

	/*
	 * We have to make sure the interrupt is masked regardless
	 * of our on_list status.  This is because of a window where
	 * a shared (spurious) interrupt comes after on_list
	 * is knocked down but before the enable() callout is made.
	 * If enable() then happened to run after we masked, we
	 * could end up on the list without the interrupt masked
	 * which would cause the kernel more than a little grief
	 * if one of our real interrupts then came in.
	 *
	 * This window doesn't exist when using kermask since the
	 * interrupt isn't unmasked until all the enable()s run
	 * (mask count is tracked by kernel).
	 */

	/*
	 * If this was controling real hardware, mask of
	 * interrupts here. eg from i82544 driver:
	 */
	i82544->reg[I82544_IMC] = 0xffffffff;

	return interrupt_queue(sam->sc_iopkt, ient);
}
#endif
int
sam_process_interrupt(void *arg, struct nw_work_thread *wtp)
{
	printf("In sam_process_interrupt\n");
	struct sam_dev		*sam;
	struct mbuf			*m;
	struct ifnet		*ifp;
	struct ether_header	*eh;

	sam = arg;
	ifp = &sam->sc_ec.ec_if;


	if ((sam->sc_intr_cnt++ % 1000) == 0) {
		/* Send a packet up */
		m = m_getcl_wtp(M_DONTWAIT, MT_DATA, M_PKTHDR, wtp);

		if (!m) {
            ifp->if_ierrors++;  // for ifconfig -v
			return 1;
		}

		m->m_pkthdr.len = m->m_len = sizeof(*eh);

		// ip_input() needs this
		m->m_pkthdr.rcvif = ifp;

		// dummy up a broadcasted IP packet for testing
		eh = mtod(m, struct ether_header *);
		eh->ether_type = ntohs(ETHERTYPE_IP);
		memcpy(eh->ether_dhost, etherbroadcastaddr, ETHER_ADDR_LEN);

		ifp->if_ipackets++; // for ifconfig -v

		(*ifp->if_input)(ifp, m);

		printf("sam_process_interrupt %d\n", sam->sc_intr_cnt);
	}

	/*
	 * return of 1 means were done.
	 *
	 * If we notice we're taking a long time (eg. processed
	 * half our rx descriptors) we could early out with a
	 * return of 0 which lets other interrupts be processed
	 * without calling our interrupt_enable func.  This
	 * func will be called again later.
	 */
	return 1;
}
#ifndef HW_MASK
int
sam_enable_interrupt(void *arg)
{
	printf("In sam_enable_interrupt\n");
	struct sam_dev	*sam;

	sam = arg;
	InterruptUnmask(sam->sc_irq, sam->sc_iid);

	return 1;
}
#else
int
sam_enable_interrupt(void *arg)
{
	struct sam_dev	*sam;

	sam = arg;
	/* eg from i82544 driver */

	i82544->reg[I82544_IMS] = i82544->intrmask;

	return 1;
}
#endif



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL$ $Rev$")
#endif
