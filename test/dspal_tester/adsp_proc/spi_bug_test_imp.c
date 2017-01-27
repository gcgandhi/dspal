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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdbool.h>
#include <dev_fs_lib_spi.h>
#include <test_status.h>
#include <test_utils.h>
#include "platform.h"
#include <errno.h>
#include <sys/time.h>


int dspal_tester_spi_relay_open(int device) {

    int fd;

    char devName[256];
    snprintf(devName,sizeof(devName),DEV_FS_SPI_DEVICE_TYPE_STRING "%d",device);
    // null terminate
    devName[sizeof(devName)-1] = 0;
    LOG_INFO("Opening SPI device %s",devName);
    fd = open(devName, O_RDWR);
    if (fd == -1) {
        LOG_ERR("open SPI device %d failed. %d: %s",device,errno,strerror(errno));
        return errno;
    } else {
        LOG_INFO("Successfully opened SPI device %s fd %d",devName,fd);
        return fd;
    }

}

int dspal_tester_spi_relay_read_write(int fd, const unsigned char* write_data, int write_dataLen, unsigned char* read_data, int read_dataLen) {

	struct dspal_spi_ioctl_read_write read_write;

	read_write.read_buffer = (void*) read_data;
	read_write.read_buffer_length = read_dataLen;
	read_write.write_buffer = (void*) write_data;
	read_write.write_buffer_length = write_dataLen;

	unsigned int byte;
    for (byte = 0; byte < read_write.read_buffer_length; byte++) {
    	read_data[byte] = 0xA5;
    }

    // We must update the slave address before/after writing to get the chip
    // select behavior desired for the FPGA:
	struct dspal_spi_ioctl_set_options options = {
		.slave_address = 0,
		.is_tx_data_synchronous = 0,
		.tx_data_callback = 0,
		.rx_data_callback = 0,
	};

	int result = ioctl(fd, SPI_IOCTL_SET_OPTIONS, &options);

	if (result < SUCCESS) {
		LOG_ERR("SPI %d slave set 1 error! %d: %s",fd,errno,strerror(errno));
		return errno;
	}

	// For some reason we need to re-assert this every write.  Otherwise, we
	// see the clk line go idle high after the first write.
    struct dspal_spi_ioctl_set_spi_mode mode = {
    		.eClockPolarity = SPI_CLOCK_IDLE_LOW,
			.eShiftMode = SPI_INPUT_FIRST,
    };

    // configure SPI mode:
    if (ioctl(fd, SPI_IOCTL_SET_SPI_MODE, (void *)&mode) != SUCCESS) {
        LOG_ERR("ioctl SPI SET MODE 1 fd %d failed. %d: %s",fd,errno,strerror(errno));
        return errno;
    }

    // Finally, we can write:
    LOG_INFO("Writing %d bytes to SPI",read_write.write_buffer_length);

	result = ioctl(fd, SPI_IOCTL_RDWR, &read_write);

	if (result < SUCCESS) {
		LOG_ERR("SPI %d read/write error! %d: %s",fd,errno,strerror(errno));
		return errno;
	}

	// Once again to get the desired chip select behavior after writing:
	options.slave_address = 1;

	result = ioctl(fd, SPI_IOCTL_SET_OPTIONS, &options);

	if (result < SUCCESS) {
		LOG_ERR("SPI %d slave set 2 error! %d: %s",fd,errno,strerror(errno));
		return errno;
	}

    // Once again re-asserting the SPI mode, so that the clk is idle low after writing:
    if (ioctl(fd, SPI_IOCTL_SET_SPI_MODE, (void *)&mode) != SUCCESS) {
        LOG_ERR("ioctl SPI SET MODE 2 fd %d failed. %d: %s",fd,errno,strerror(errno));
        return errno;
    }

#if 1
	LOG_INFO("SPI: ");
    for (byte = 0; byte < read_write.write_buffer_length; byte++) {
    	LOG_INFO("write_data: 0x%02X ",write_data[byte]);
    }
    for (byte = 0; byte < read_write.read_buffer_length; byte++) {
    	LOG_INFO("read_data: 0x%02X ",read_data[byte]);
    }
    LOG_INFO("\n");
#endif

	return SUCCESS;
}

int dspal_tester_spi_relay_configure(int fd, int clock) {

    struct dspal_spi_ioctl_set_bus_frequency rate = {
            .bus_frequency_in_hz = clock
        };

	struct dspal_spi_ioctl_loopback loopback;


    // configure SPI clock rate
    if (ioctl(fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, (void *)&rate) != SUCCESS) {
        LOG_ERR("ioctl SPI SET FREQ fd %d failed. %d: %s",fd,errno,strerror(errno));
        return errno;
    } else {
        LOG_INFO("SPI fd %d freq successfully configured to %d",fd,clock);
    }

    struct dspal_spi_ioctl_set_spi_mode mode = {
    		.eClockPolarity = SPI_CLOCK_IDLE_LOW,
			.eShiftMode = SPI_INPUT_FIRST,
    };

    // configure SPI clock rate
    if (ioctl(fd, SPI_IOCTL_SET_SPI_MODE, (void *)&mode) != SUCCESS) {
        LOG_ERR("ioctl SPI SET MODE fd %d failed. %d: %s",fd,errno,strerror(errno));
        return errno;
    } else {
        LOG_INFO("SPI fd %d mode successfully configured",fd);
    }

	loopback.state = SPI_LOOPBACK_STATE_DISABLED;

	int result = ioctl(fd, SPI_IOCTL_LOOPBACK_TEST, &loopback);

	if (result < SUCCESS) {
		LOG_ERR("error: unable to activate spi %d loopback mode. %d: %s",fd,errno,strerror(errno));
		return errno;
	} else {
		LOG_INFO("%s SPI loopback mode",loopback.state == SPI_LOOPBACK_STATE_DISABLED?"Disabled":"Enabled");
	}

	// Updating the slave address to get the desired chip select behavior:
	struct dspal_spi_ioctl_set_options options = {
		.slave_address = 1,
		.is_tx_data_synchronous = 0,
		.tx_data_callback = 0,
		.rx_data_callback = 0,
	};

	result = ioctl(fd, SPI_IOCTL_SET_OPTIONS, &options);

	if (result < SUCCESS) {
		LOG_ERR("SPI %d slave set error! %d: %s",fd,errno,strerror(errno));
		return errno;
	}

	return SUCCESS;
}

int dspal_tester_spi_relay_close(int fd) {
    LOG_INFO("Closing SPI device %d",fd);
    return close(fd);
}

/**
* @brief Test SPI bug w/ garbage data on first transaction
*
* @return
* SUCCESS  ------ Test Passes
* ERROR ------ Test Failed
*/
int dspal_spi_bug_test(void)
{

	int result = SUCCESS;
	// Issue occurs more often with larger buffer sizes it seems
	const unsigned char write_data[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,28,19,20,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,28,19,20};
	unsigned char read_data[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	// Open, configure, write data, then close:
	// Note, cannot easily reproduce the bug by doing this in a loop.  Easier just to run this
	// test multiple times.  Typically happens w/in 30 tries.
	int fd = dspal_tester_spi_relay_open(8); // device 8
	int res = dspal_tester_spi_relay_configure(fd,1000000); // 1 Mhz
	res = dspal_tester_spi_relay_read_write(fd, write_data, 40, read_data, 40);
	res = dspal_tester_spi_relay_close(fd);

	return result;
}

/**
 * Main entry point for the SPI automated test.
 * @return
 * - ERROR: Indicates that the test has failed.
 * - SUCCESS: Test has passed
 */
int dspal_tester_spi_bug_test(void)
{
	int result;

	LOG_INFO("beginning spi bug test");

	if ((result = dspal_spi_bug_test()) < SUCCESS) {
		LOG_ERR("error: spi bug test failed: %d", result);
		return result;
	}

	return SUCCESS;
}





