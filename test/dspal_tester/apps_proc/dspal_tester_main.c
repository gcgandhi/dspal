/****************************************************************************
 *   Copyright (c) 2015 James Wilson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <getopt.h>
#include <time.h>

#include "adspmsgd.h"
#include "rpcmem.h"

#include "test_utils.h"
#include "dspal_tester.h"
#include "posix_test_suite.h"
#include "io_test_suite.h"
#include <sys/time.h>

//#include <dev_fs_lib_serial.h>  // this doesnt work....


// Borrowed from DSP serial header

enum DspUartRates {
    UART_BITRATE_ILLEGAL_1,            /**< Illegal bit-rate 1*/
    UART_BITRATE_ILLEGAL_3,            /**< Illegal bit-rate 2*/
    UART_BITRATE_ILLEGAL_4,            /**< Illegal bit-rate 4*/
    UART_BITRATE_ILLEGAL_5,            /**< Illegal bit-rate 5*/
    UART_BITRATE_ILLEGAL_6,            /**< Illegal bit-rate 6*/
    UART_BITRATE_ILLEGAL_7,            /**< Illegal bit-rate 7*/
    UART_BITRATE_300,                  /**< 300  bit-rate     */
    UART_BITRATE_600,                  /**< 600  bit-rate     */
    UART_BITRATE_1200,                 /**< 1200 bit-rate     */
    UART_BITRATE_2400,                 /**< 2400 bit-rate     */
    UART_BITRATE_4800,                 /**< 4800 bit-rate     */
    UART_BITRATE_9600,                 /**< 9600 bit-rate     */
    UART_BITRATE_14400,                /**< 14400 bit-rate    */
    UART_BITRATE_19200,                /**< 19200  bit-rate   */
    UART_BITRATE_38400,                /**< 38400  bit-rate   */
    UART_BITRATE_57600,                /**< 57600  bit-rate   */
    UART_BITRATE_76800,                /**< 76800  bit-rate   */
    UART_BITRATE_115200,               /**< 115200 bit-rate   */
    UART_BITRATE_230400,               /**< 230400 bit-rate   */
    UART_BITRATE_250000,
    UART_BITRATE_460800,               /**< 460800 bit-rate   */
    UART_BITRATE_921600,               /**< 921600 bit-rate   */
    UART_BITRATE_2000000,              /**< 2000000 bit-rate  */
    UART_BITRATE_2900000,              /**< 2900000 bit-rate  */
    UART_BITRATE_3000000,              /**< 3000000 bit-rate  */
    UART_BITRATE_3200000,              /**< 3200000 bit-rate  */
    UART_BITRATE_3686400,              /**< 3686400 bit-rate  */
    UART_BITRATE_4000000              /**< 4000000 bit-rate  */
};

static unsigned char buffer_gbl[512*1024]; // Garbage data

void spi_bug_test_main()
{

	// Bug occurs both if sending from APP or DSP side:
#if 1
	LOG_INFO("Sending SPI Data from APP");

	// Bug occurs if this is taken off the stack as well
	// Issue occurs more often with larger buffer sizes it seems
	const unsigned char write_data[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,28,19,20,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,28,19,20};
	unsigned char read_data[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	// Open, configure, write data, then close:
	// Note, cannot easily reproduce the bug by doing this in a loop.  Easier just to run this
	// test multiple times.  Typically happens w/in 30 tries.
	int fd = dspal_tester_spi_relay_open(8); // device 8
	int res = dspal_tester_spi_relay_configure(fd,1000000); // 1 Mhz
	// Bug doesnt occur if I dont write
#if 1
	res = dspal_tester_spi_relay_read_write(fd, write_data, 40, read_data, 40);
#endif
	res = dspal_tester_spi_relay_close(fd);
#else
	LOG_INFO("Sending SPI Data from DSP");

	dspal_tester_spi_bug_test();
#endif
}

void uart_bug_test_main()
{

	// Bug occurs sending just from the DSP side, or from the APP side
#if 0

	printf("Sending UART data from DSP \n\n");

	dspal_tester_uart_bug_test();

#else
	int use_rpcmem = 1;

	printf("Sending UART data from APP (%d)\n\n",use_rpcmem);

	// Open, configure:
	int device = 2;
	int fd = dspal_tester_uart_relay_open(device);
	// Problem occurs regardless of bit rate:
	int res = dspal_tester_uart_relay_configure(fd,device,UART_BITRATE_230400,0,0,0); // 115K baud

	// Write in 256 bytes chunks:
	const int xferSize = 4*512*1024;  // 512 KB, which is what we are trying to send
	const int WRITE_BUFF_SIZE = 256; // Increasing/decreasing this size doesnt fix the issue
	unsigned char buffer[xferSize]; // Garbage data
	unsigned char* data;
	unsigned char* data_rpc;

	// With rpcmem
	// It performs a little bit better with rpcmem, but still has the bug
	if (use_rpcmem) {
		rpcmem_init();
		// Using heap of default or 22 didnt make a difference
		// Using flags of UNCACHED or DEFAULT didnt make a difference
		data_rpc = (unsigned char*)rpcmem_alloc(RPCMEM_DEFAULT_HEAP, RPCMEM_FLAG_UNCACHED, xferSize);
		memcpy(data_rpc, buffer_gbl /*buffer*/, xferSize);
		data = data_rpc;
	}
	// Without rpcmem:
	else {
		data = buffer_gbl; //buffer;
	}

	int chunk;
	int iter = 0;

	// Bug is seen when uart writes return bad status....
	LOG_INFO("Writing a total of %d bytes", xferSize);

	int ii = 0;
	for (ii = 0; ii < 3; ++ii) {

		for (chunk = 0; chunk < xferSize; chunk+=WRITE_BUFF_SIZE) {

			//printf("iter=%d, chunk=%d \n", ++iter, chunk);
			int left = xferSize - chunk;
			int thisSize = WRITE_BUFF_SIZE < left ? WRITE_BUFF_SIZE : left;

			int stat = dspal_tester_uart_relay_write(fd,device,data+chunk,thisSize);
			if (stat == -1) {
				LOG_ERR("BAD UART WRITE (iter=%d, chunk=%d)!!!!\n", iter, chunk);
				break;
			}
			//data += chunk; // TODO bug
			iter++;

			// Even with the 1 sec delay, the bug still occurs.
			//usleep(1000000); // 1 sec delay
		}
	}

	// Close:
    if (use_rpcmem) {
        rpcmem_free(data_rpc);
    }
    res = dspal_tester_uart_relay_close(fd,device);

#endif

}

void uart_stress_test()
{

	/*
	 * Simulate NAV to FC communication
	 * 1 Mbps baud rate, 128 bytes, 2 ms period, sleep 2/3 ms
	 *
	 */
	LOG_INFO("UART stress test");

	// Open, configure:
	int device = 2;
	int fd = dspal_tester_uart_relay_open(device);
	int res = dspal_tester_uart_relay_configure(fd,device,UART_BITRATE_921600,0,0,0); // 115K baud

	int device2 = 3;
	int fd2 = dspal_tester_uart_relay_open(device2);
	int res2 = dspal_tester_uart_relay_configure(fd2,device2,UART_BITRATE_921600,0,0,0); // 115K baud

	// Write in 128 bytes chunks:
	const int xferSize = 124;  // 5 KB
	const int WRITE_BUFF_SIZE = 256;
	unsigned char buffer[xferSize]; // Garbage data
	unsigned char* data;
	unsigned char* data_rpc;

	rpcmem_init();
	data_rpc = (unsigned char*)rpcmem_alloc(RPCMEM_DEFAULT_HEAP, RPCMEM_FLAG_UNCACHED, xferSize);
	memcpy(data_rpc, buffer, xferSize);

	int i;
	for (i = 0; i < 10; ++i) {

		int chunk;
		int iter = 0;
		data = data_rpc;

		for (chunk = 0; chunk < xferSize; chunk+=WRITE_BUFF_SIZE) {

			int left = xferSize - chunk;
			int thisSize = WRITE_BUFF_SIZE < left ? WRITE_BUFF_SIZE : left;

#if 1
	        struct timespec stime_init;
			clock_gettime(CLOCK_REALTIME, &stime_init);

			int stat = dspal_tester_uart_relay_write(fd,device,data,thisSize);

		    struct timespec stime_after;
			clock_gettime(CLOCK_REALTIME, &stime_after);

//			LOG_INFO("<<< init.ns: %d, init:s: %d, after.ns: %d, after.s: %d",
//					 stime_init.tv_nsec, stime_init.tv_sec,
//					 stime_after.tv_nsec, stime_after.tv_sec);
			LOG_INFO("<<< diff.s: %d, diff.ns: %d",
					stime_after.tv_sec - stime_init.tv_sec,
					stime_after.tv_nsec - stime_init.tv_nsec);

#else
			int stat = dspal_tester_uart_relay_write(fd,device,data,thisSize);
#endif

			if (stat == -1) {
				LOG_ERR("BAD UART1 WRITE (iter: %d)!!!!\n", i);
				break;
			}

#if 0
			stat = dspal_tester_uart_relay_write(fd2,device2,data,thisSize);
			if (stat == -1) {
				LOG_ERR("BAD UART2 WRITE (iter: %d)!!!!\n", i);
				break;
			}
#endif
			data += chunk;
		}

#if 0
		spi_bug_test_main();
#endif

        //usleep(389); // delay

	}

	// Close:
    rpcmem_free(data_rpc);
    res = dspal_tester_uart_relay_close(fd,device);
}

int main(int argc, char *argv[])
{

	// Stress test performs w/ no issues
#if 0
	uart_stress_test();
#endif

	// UART bug is now fixed
#if 0
	uart_bug_test_main();
#endif

#if 1
	spi_bug_test_main();
#endif

	return 0;
}

