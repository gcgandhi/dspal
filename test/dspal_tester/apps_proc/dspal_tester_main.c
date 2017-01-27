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

#include "adspmsgd.h"
#include "rpcmem.h"

#include "test_utils.h"
#include "dspal_tester.h"
#include "posix_test_suite.h"
#include "io_test_suite.h"
#include <sys/time.h>



int main(int argc, char *argv[])
{
	// Bug occurs if this is taken off the stack as well
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

	return 0;
}

