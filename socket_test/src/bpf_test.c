/*
 * bpf_test.c
 *
 *  Created on: Apr 16, 2021
 *      Author: jbagi
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <errno.h>
#include <fcntl.h>
#include <net/if_ether.h>
#include <net/if.h>
#include <net/bpf.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>


static char bpf_dev[] = "/dev/bpf0";
int bpf = 0;
typedef unsigned int uint32;
typedef int sint32;

int Mw_TL_OpenBPFDevice() {
	int RetVal = -1;
	struct ifreq bpf_bound_if;                 // interface name (ravb)
	// size of buffer of all raw ethernet frames + bpf packet header
	int Mw_TL_MaxBPFBufferlength = 1000; /* PRQA S 0306, 1840 */ /* Intended conversion and arithmetic operation */

	uint32 bpf_rx_buf_len_readback = 0u;

	sint32 bpf_immediate_mode = 1;

	// Copy own interface name that will be used by the bpf to send/receive
	(void) strncpy(bpf_bound_if.ifr_name, "sam0\0", 5);

	/* BPF setup for raw Eth frame transmission */
	// open file descriptor for device
	bpf = open(bpf_dev, O_RDWR); /* PRQA S 0339 */ /* Allow octal constants for sys/fcntl.h definition */
	if (bpf < 0) {
	  perror("Failed opening /dev/bpf0");
	  (void) close(bpf);
	  bpf = -1;
	  // return immediately upon packet read
	} else if (ioctl(bpf, BIOCIMMEDIATE, &bpf_immediate_mode) < 0) { /* PRQA S 2855 */ /* Controlled behaviour */
	  perror("Can't set IMMEDIATE mode on bpf device");
	  (void) close(bpf);
	  bpf = -1;
	  // set BPF buffer length
	} else if (ioctl(bpf, BIOCSBLEN, (caddr_t)&Mw_TL_MaxBPFBufferlength) < 0) {/* PRQA S 2855 */ /* Controlled behaviour */
	  perror("Can't set buffer length of bpf device");
	  (void) close(bpf);
	  bpf = -1;
	} else if (ioctl(bpf, BIOCGBLEN, &bpf_rx_buf_len_readback) < 0) { // read back buffer length
	  perror("Can't get buffer length of bpf device");
	  (void) close(bpf);
	  bpf = -1;
	} else if (bpf_rx_buf_len_readback != Mw_TL_MaxBPFBufferlength) { // check readback value of what was set
	  (void) close(bpf);
	  bpf = -1;
	  printf("bpf rx buffer length readback value is %d instead of %d",
				 bpf_rx_buf_len_readback, Mw_TL_MaxBPFBufferlength);
	  // bind bpf device to ravb interface
	} else if (ioctl(bpf, BIOCSETIF, &bpf_bound_if) < 0) { /* PRQA S 2855 */ /* Controlled behaviour */
	  perror("Can't bind bpf to interface");
	  (void) close(bpf);
	  bpf = -1;
	} else if (ioctl(bpf, BIOCPROMISC, NULL) < 0) { // all packets processed on the interface
	  perror("Can't set device in promiscous mode");
	  (void) close(bpf);
	  bpf = -1;
	// set bpf to non blocking mode
	} else if (fcntl(bpf, F_SETFL, O_NONBLOCK) != 0) { /* PRQA S 0339 */ /* Allow octal constants for sys/fcntl.h definition */
	  perror("Can't set bpf in non-blocking mode");
	  (void) close(bpf);
	  bpf = -1;
	}/* else if (set_bpffilter() != 0U) { // set bpf read filter
	  printf ("Could not set read filter for bpf [%d]\n%s",
				  errno, strerror (errno) );
	  (void) close(bpf);
	  bpf = -1;
	}*/ else {  // all ioctl/fcntl ok
	  RetVal = 0;
	}

  return bpf;
}
