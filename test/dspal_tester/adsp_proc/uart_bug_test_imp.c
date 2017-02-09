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

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <mqueue.h>
#include <semaphore.h>

#include <dev_fs_lib_serial.h>
#include <termios.h>

#include <test_status.h>
#include <test_utils.h>

#include <assert.h>


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

// A message queue per potential UART - indexed by device number
#define NUM_UARTS (6)

// double buffers for received data
static struct UartRecvBufferEntry {
    int active; // active buffer for receives
    unsigned char* buffPtrs[2]; // double buffers
    size_t recvSize[2]; // received data size
    size_t buffSize; // buffer size
    sem_t semId; // semaphore for signaling waiting thread
    unsigned int buffWritePos[2];// Write position of circular buffer, ie tail
    unsigned int buffReadPos[2];	// Read position of circular buffer, ie head
    int quitRequested; // used to signal exit of semaphore waits
} UartReceiveBuffers[NUM_UARTS];
;

static float portBitTimes[NUM_UARTS];

long dspal_tester_uart_relay_quit(int device) {

    UartReceiveBuffers[device].quitRequested = 1;

    // signal waiting thread
    int sem_cnt;
    (void) sem_getvalue(&UartReceiveBuffers[device].semId, &sem_cnt);

    // TODO hack to make act like binary semaphore b/c there is none available:
    // Note, dont want the counting semaphore b/c this callback will be called many times before the
    // read thread runs
    if (sem_cnt == 0) {
        int stat = sem_post(&UartReceiveBuffers[device].semId);

        if (-1 == stat) {
            LOG_ERR("Error posting semaphore");
        }
    }

    LOG_INFO("UART device %d quit requested", device);

    return 0;

}

long dspal_tester_uart_receive_allocate(int device, int size) {

    assert(device < NUM_UARTS);

    LOG_INFO("Allocating buffers for UART %d size %d\n", device, size);

    UartReceiveBuffers[device].active = 0;
    UartReceiveBuffers[device].buffPtrs[0] = malloc(size);
    UartReceiveBuffers[device].buffPtrs[1] = malloc(size);
    UartReceiveBuffers[device].recvSize[0] = 0;
    UartReceiveBuffers[device].recvSize[1] = 0;
    UartReceiveBuffers[device].buffWritePos[0] = 0;
    UartReceiveBuffers[device].buffWritePos[1] = 0;
    UartReceiveBuffers[device].buffReadPos[0] = 0;
    UartReceiveBuffers[device].buffReadPos[1] = 0;
    UartReceiveBuffers[device].buffSize = size;
    UartReceiveBuffers[device].quitRequested = 0;
    int stat = sem_init(&UartReceiveBuffers[device].semId, 0, 0);
    if (-1 == stat) {
        LOG_ERR("Error initializing UART %d semaphore", device);
        return -1;
    };

    if ((0 == UartReceiveBuffers[device].buffPtrs[0]
            || 0 == UartReceiveBuffers[device].buffPtrs[1])) {
        LOG_ERR("Unable to allocate memory for UART %i", device);
        return -1;
    } else {
        return 0;
    }

}

// This callback will have a race condition. The copy back to the app processor will have to happen before the
// next transfer happens.

void dspal_tester_uart_cb(void *context, char *buffer, size_t num_bytes) {
    int device = (int) context;

    if (device >= NUM_UARTS) {
        LOG_ERR("Invalid device %i cb!", device);
        return;
    }

    LOG_INFO("UART received %d bytes", num_bytes);

    // Ring buffer:
    // TODO this is not thread safe, buffReadPos is updated by the read thread, but I dont "think" that thread
    // can ever pre-empt this call back which we believe is in interrupt context....
#if 1
    unsigned int readPos =
            UartReceiveBuffers[device].buffReadPos[UartReceiveBuffers[device].active];

    // Error if overflowing buffer for now:
    unsigned int buff_size =
            (UartReceiveBuffers[device].buffWritePos[UartReceiveBuffers[device].active]
                    - readPos + UartReceiveBuffers[device].buffSize)
                    % UartReceiveBuffers[device].buffSize;
    if ((buff_size + num_bytes) > UartReceiveBuffers[device].buffSize) {
        LOG_ERR(
                "Device %d had a buffer overflow with readPos: %u, writePos: %u, buffCapcity: %u",
                device, readPos,
                UartReceiveBuffers[device].buffWritePos[UartReceiveBuffers[device].active],
                UartReceiveBuffers[device].buffSize);
        return;
    }

    // copy data into active buffer
    // If there is no wrap around:
    if ((UartReceiveBuffers[device].buffWritePos[UartReceiveBuffers[device].active]
            + num_bytes) <= UartReceiveBuffers[device].buffSize) {

        memcpy(
                UartReceiveBuffers[device].buffPtrs[UartReceiveBuffers[device].active]
                        + UartReceiveBuffers[device].buffWritePos[UartReceiveBuffers[device].active],
                buffer, num_bytes);
    } else {
        unsigned int size_till_end =
                UartReceiveBuffers[device].buffSize
                        - UartReceiveBuffers[device].buffWritePos[UartReceiveBuffers[device].active];

        // Copy till end of buffer:
        memcpy(
                UartReceiveBuffers[device].buffPtrs[UartReceiveBuffers[device].active]
                        + UartReceiveBuffers[device].buffWritePos[UartReceiveBuffers[device].active],
                buffer, size_till_end);

        unsigned int wrap_around_size = num_bytes - size_till_end;

        // Copy wrapped around buffer:
        if (wrap_around_size > 0) {
            memcpy(
                    UartReceiveBuffers[device].buffPtrs[UartReceiveBuffers[device].active],
                    buffer + size_till_end, wrap_around_size);
        }
    }

    UartReceiveBuffers[device].buffWritePos[UartReceiveBuffers[device].active] =
            (UartReceiveBuffers[device].buffWritePos[UartReceiveBuffers[device].active]
                    + num_bytes) % UartReceiveBuffers[device].buffSize;

//	struct timespec stime;
//	(void)clock_gettime(CLOCK_REALTIME,&stime);
//	LOG_INFO("<<<< cb: active: %d at %d %d\n", UartReceiveBuffers[device].active, stime.tv_sec, stime.tv_nsec);

    // signal waiting thread
    int sem_cnt;
    (void) sem_getvalue(&UartReceiveBuffers[device].semId, &sem_cnt);

    // TODO hack to make act like binary semaphore b/c there is none available:
    // Note, dont want the counting semaphore b/c this callback will be called many times before the
    // read thread runs
    if (sem_cnt == 0) {
        int stat = sem_post(&UartReceiveBuffers[device].semId);

        if (-1 == stat) {
            LOG_ERR("Error posting semaphore");
        }
    }

    // Ping-pong buffers:
#else
    if (num_bytes > UartReceiveBuffers[device].buffSize) {
        LOG_ERR("Buffer for UART %d too big. Received: %d Alloc: %d",device,num_bytes, UartReceiveBuffers[device].buffSize);
        return;
    }

    // verify active buffer
    if (UartReceiveBuffers[device].buffPtrs[UartReceiveBuffers[device].active] == 0) {
        LOG_ERR("Device %d receive buffers unallocated",device);
        return;
    }

    // copy data into active buffer
    memcpy(UartReceiveBuffers[device].buffPtrs[UartReceiveBuffers[device].active],buffer,num_bytes);
    UartReceiveBuffers[device].recvSize[UartReceiveBuffers[device].active] = num_bytes;

    // flip buffers
    UartReceiveBuffers[device].active = 1 - UartReceiveBuffers[device].active;

//    struct timespec stime;
//    (void)clock_gettime(CLOCK_REALTIME,&stime);
//    LOG_INFO("<<<< cb: active: %d at %d %d\n", UartReceiveBuffers[device].active, stime.tv_sec, stime.tv_nsec);

    // signal waiting thread
    int stat = sem_post(&UartReceiveBuffers[device].semId);

    if (-1 == stat) {
        LOG_ERR("Error posting semaphore");
    }
#endif

#if 0
    // send message with UART data - FIXME: not very efficient for large transfers
    int stat = mq_send(UartRecvQueues[device],buffer,num_bytes,0);
    if (-1 == stat) {
        LOG_ERR("mq_send error: %s",strerror(errno));
        return;
    }
#endif

}

int dspal_tester_uart_relay_open(int uart) {

    int fd;

    char devName[256];
    snprintf(devName, sizeof(devName), DEV_FS_UART_DEVICE_TYPE_STRING "%d",
            uart);
    // null terminate
    devName[sizeof(devName) - 1] = 0;
    LOG_INFO("Opening UART device %s", devName);
    fd = open(devName, O_RDWR | O_NONBLOCK | O_SYNC);
    if (fd == -1) {
        LOG_ERR("open UART device %d failed.", uart);
        return ERROR;
    } else {
        LOG_INFO("Successfully opened UART device %s fd %d", devName, fd);
    }

#if 0
    // create message queue
    char qName[256];
    // borrow devName
    snprintf(devName,sizeof(devName),"/uartq-%d",uart);
    // null terminate
    devName[sizeof(devName)-1] = 0;
    struct mq_attr qattr;
    qattr.mq_flags = 0;
    qattr.mq_maxmsg = 5;
    qattr.mq_msgsize = 1024;
    qattr.mq_curmsgs = 0;

    UartRecvQueues[uart] = mq_open(devName,O_RDWR|O_CREAT,0,&qattr);
    if (-1 == UartRecvQueues[uart]) {
        LOG_ERR("Error opening msg q for device %d (%s): %s",uart,devName,strerror(errno));
        close(fd);
        return -1;
    }
#endif

#if 1
    // configure read callback
    struct dspal_serial_ioctl_receive_data_callback receive_callback;
    receive_callback.rx_data_callback_func_ptr = dspal_tester_uart_cb;
    receive_callback.context = (void*) uart;

    int stat = ioctl(fd, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,
            (void *) &receive_callback);

    if (-1 == stat) {
        LOG_ERR("UART callback error: %d", strerror(errno));
        close(fd);
        return -1;
    }
#endif

#if 0
    // configure blocking reads
    struct termios cfg;
    int stat = tcgetattr(fd,&cfg);
    if (-1 == stat) {
        LOG_ERR("tcgetattr failed: (%d): %s",stat,strerror(errno));
        close(fd);
        return -1;
    } else {
        LOG_INFO("tcgetattr passed.");
    }
    // wait for bytes for 1 second
    cfg.c_cc[VMIN] = 0;
    cfg.c_cc[VTIME] = 10;

    stat = tcsetattr(fd,0,&cfg);
    if (-1 == stat) {
        LOG_ERR("tcsetattr failed: (%d): %s",stat,strerror(errno));
        close(fd);
        return -1;
    } else {
        LOG_INFO("tcsetattr passed.");
    }
#endif

    return fd;
}

int dspal_tester_uart_check_devs(void) {

    for (int uart = 1; uart < 10; uart++) {

        char devName[256];
        snprintf(devName, sizeof(devName), DEV_FS_UART_DEVICE_TYPE_STRING "%d",
                uart);
        // null terminate
        devName[sizeof(devName) - 1] = 0;
        LOG_INFO("Opening UART device %s", devName);
        int fd = open(devName, 0);
        if (fd == -1) {
            LOG_ERR("open UART device %d failed.", uart);
        } else {
            LOG_INFO("Successfully opened UART device %s fd %d", devName, fd);
        }
        if (fd != -1) {
            close(fd);
        }

    }

    return 0;

}

int dspal_tester_uart_relay_configure(int fd, int device, int baud, int parity,
        int bits, int stop_bits) {

    struct dspal_serial_ioctl_data_rate rate = { .bit_rate = baud };

    // configure GPIO device into general purpose IO mode
    if (ioctl(fd, SERIAL_IOCTL_SET_DATA_RATE, (void *) &rate) != SUCCESS) {
        LOG_ERR("ioctl UART fd %d failed", fd);
        return ERROR;
    } else {
        LOG_INFO("UART fd %d successfully configured", fd);
    }

    // save bit time in usec
    switch (baud) {
        case DSPAL_SIO_BITRATE_115200:
            portBitTimes[device] = 0.0000086805556 * 1000000.0 * 10;
            break;
        case DSPAL_SIO_BITRATE_230400:
            portBitTimes[device] = 0.0000043402778 * 1000000.0 * 10;
            break;
        case DSPAL_SIO_BITRATE_921600:
            portBitTimes[device] = 0.0000010850694 * 1000000.0 * 10;
            break;
        default:
            return ERROR;
    }

    return SUCCESS;
}

int dspal_tester_uart_relay_read(int device, unsigned char* buff, int buffLen,
        int * bytes) {

    if (device >= NUM_UARTS) {
        LOG_ERR("Requested device id %d too large", device);
        return -1;
    }

    // check for pending quit before waiting
    if (1 == UartReceiveBuffers[device].quitRequested) {
        return 1;
    }

#if 0
    // have wait timeout for app processor to exit if it needs
    struct timespec ts;
    ts.tv_sec = 1;
    ts.tv_nsec = 0;

    // wait for next transaction
    int stat = sem_timedwait(&UartReceiveBuffers[device].semId,&ts);
    if (-1 == stat) {
        // check for timout to return control to app processor driver
        if (ETIMEDOUT == errno) {
            return 1;
        } else { // other error
            return -1;
        }
    }
#else

    int stat = sem_wait(&UartReceiveBuffers[device].semId);
    if (-1 == stat) {
        return -1;
    }

    // check for pending quit
    if (1 == UartReceiveBuffers[device].quitRequested) {
        LOG_INFO("UART device %d quitting.", device);
        return 1;
    }

#endif
    // Ring buffer:
    // TODO This is not thread safe, buffWritePos can be updated by the UART callback, which we believe is
    // called in interrupt context.  Thus, the callback can occur during the execution of this.  However,
    // the writePos being updated during this execution would just mean that not all the data is copied this
    // time, which is okay...
#if 1

    int readBuffer = UartReceiveBuffers[device].active;

    unsigned int writePos = UartReceiveBuffers[device].buffWritePos[readBuffer];
    size_t bytes_recvd = (writePos
            - UartReceiveBuffers[device].buffReadPos[readBuffer]
            + UartReceiveBuffers[device].buffSize)
            % UartReceiveBuffers[device].buffSize;

    // Buffer empty:
    if (bytes_recvd == 0) {
        LOG_ERR("Empty receive buffer");
        return -1;
    }

    if (bytes_recvd > (size_t) buffLen) {
        LOG_ERR("Receive data larger than buffer. Recv: %d Buff: %d",
                bytes_recvd, buffLen);
        return -1;
    }

//	struct timespec stime;
//	(void)clock_gettime(CLOCK_REALTIME,&stime);
//	LOG_INFO("<<<! readBuffer: %u at time %d %d\n", readBuffer, stime.tv_sec, stime.tv_nsec);

    // copy buffer
    // If there is no wrap around:
    if ((UartReceiveBuffers[device].buffReadPos[readBuffer] + bytes_recvd)
            <= UartReceiveBuffers[device].buffSize) {

        memcpy(buff,
                UartReceiveBuffers[device].buffPtrs[readBuffer]
                        + UartReceiveBuffers[device].buffReadPos[readBuffer],
                bytes_recvd);
    } else {
        unsigned int size_till_end = UartReceiveBuffers[device].buffSize
                - UartReceiveBuffers[device].buffReadPos[readBuffer];

        // Ideally would assert this:
        if ((size_till_end + writePos) != bytes_recvd) {

            LOG_ERR(
                    "Error with processing serial read buffer.  size_till_end: %u writePos: %u bytes_recvd: %u",
                    size_till_end, writePos, bytes_recvd);
            return -1;
        }

        // Copy till end of buffer:
        memcpy(buff,
                UartReceiveBuffers[device].buffPtrs[readBuffer]
                        + UartReceiveBuffers[device].buffReadPos[readBuffer],
                size_till_end);

        // Copy wrapped around buffer:
        if (writePos > 0) {
            memcpy(buff + size_till_end,
                    UartReceiveBuffers[device].buffPtrs[readBuffer], writePos);
        }
    }

    *bytes = bytes_recvd;

    UartReceiveBuffers[device].buffReadPos[readBuffer] =
            (UartReceiveBuffers[device].buffReadPos[readBuffer] + bytes_recvd)
                    % UartReceiveBuffers[device].buffSize;

    // Ping pong buffers:
#else
    // check non-active buffer
    int readBuffer = 1 - UartReceiveBuffers[device].active;
    size_t bytes_recvd = UartReceiveBuffers[device].recvSize[readBuffer];

    if (bytes_recvd > (size_t)buffLen) {
        LOG_ERR("Receive data larger than buffer. Recv: %d Buff: %d",bytes_recvd,buffLen);
        return -1;
    }

//    struct timespec stime;
//    (void)clock_gettime(CLOCK_REALTIME,&stime);
//	LOG_INFO("<<<! readBuffer: %u at time %d %d\n", readBuffer, stime.tv_sec, stime.tv_nsec);

    // copy buffer
    memcpy(buff,UartReceiveBuffers[device].buffPtrs[readBuffer],bytes_recvd);
    *bytes = bytes_recvd;
#endif

    return 0;

}

int dspal_tester_uart_relay_write(int fd, int device, const unsigned char* buff,
        int buffLen) {


    int written = write(fd, buff, buffLen);

    if (written != buffLen) {
        LOG_ERR("UART %d write error: %d %d", fd, buffLen, written);
        return -1;
    } else {
        //LOG_INFO("UART %d written %d bytes successfully.", fd, written);
    }
#if 0
    LOG_INFO("Drain");
    int stat = tcdrain(fd);
    if (-1 == stat) {
        LOG_ERR("UART %d tcdrain error.",fd);
        return -1;
    }
#elif 1
    float delay = (float) (buffLen) * portBitTimes[device];
    //LOG_INFO("Delay: %f", delay);
    // delay to allow transmission

//    struct timespec stime_init;
//   	clock_gettime(CLOCK_REALTIME, &stime_init);
    usleep(delay);
//    struct timespec stime_after;
//    clock_gettime(CLOCK_REALTIME, &stime_after);
//	LOG_INFO("<<< delay: diff.s: %d, diff.ns: %d",
//			stime_after.tv_sec - stime_init.tv_sec,
//			stime_after.tv_nsec - stime_init.tv_nsec);
#endif
    return 0;

}

int dspal_tester_uart_relay_close(int fd, int device) {
    LOG_INFO("Closing UART device %d", fd);
    // deallocated buffers
    if (device >= NUM_UARTS) {
        LOG_ERR("UART close invalid device %d", device);
    } else {
        free(UartReceiveBuffers[device].buffPtrs[0]);
        free(UartReceiveBuffers[device].buffPtrs[1]);
        UartReceiveBuffers[device].buffPtrs[0] =
                UartReceiveBuffers[device].buffPtrs[1] = 0;
    }

    // configure read callback
    struct dspal_serial_ioctl_receive_data_callback receive_callback;
    receive_callback.rx_data_callback_func_ptr = 0;
    receive_callback.context = 0;

    int stat = ioctl(fd, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK,
            (void *) &receive_callback);

    if (-1 == stat) {
        LOG_ERR("UART callback deregister error: %d", strerror(errno));
    }

    return close(fd);
}


/**
 * @brief Runs all the serial tests and returns 1 aggregated result.
 *
 * @return
 * SUCCESS ------ All tests pass
 * ERROR -------- One or more tests failed
 */
int dspal_tester_uart_bug_test(void)
{
    int result = SUCCESS;

	// Open, configure:
	int device = 2;
	int fd = dspal_tester_uart_relay_open(device);
	int res = dspal_tester_uart_relay_configure(fd,device,UART_BITRATE_115200,0,0,0); // 115K baud

	// Write in 256 bytes chunks:
	const int xferSize = 512*1024;  // 512 KB, which is what we are trying to send
	const int WRITE_BUFF_SIZE = 256; // Increasing/decreasing this size doesnt fix the issue
	unsigned char buffer[xferSize]; // Garbage data
	unsigned char* data;
	data = buffer;

	int chunk;
	int iter = 0;

	// Bug is seen when uart writes return bad status....
    for (chunk = 0; chunk < xferSize; chunk+=WRITE_BUFF_SIZE) {

    	LOG_INFO("iter=%d, chunk=%d \n", ++iter, chunk);
    	int left = xferSize - chunk;
        int thisSize = WRITE_BUFF_SIZE < left ? WRITE_BUFF_SIZE : left;
        int stat = dspal_tester_uart_relay_write(fd,device,data,thisSize);
        if (stat == -1) {
        	LOG_ERR("BAD UART WRITE!!!!\n");
        	result = ERROR;
        	break;
        }
        data += chunk;

        // With a 1 second delay, the board eventually resets!
        //usleep(1000000); // 1 sec delay
    }

	// Close:
    res = dspal_tester_uart_relay_close(fd,device);

    return result;
}
