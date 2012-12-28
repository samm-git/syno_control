/*
* Utility to set Synlogy DS207 "status" led according to HDD power state
* Also it is listening for key events to run USB/Power scripts.
*
* (c) Alex Samorukov 2012
*
*
* Based on: sg_io code (c) 2006-2012 Douglas Gilbert.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
* 3. The name of the author may not be used to endorse or promote products
*    derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
* SUCH DAMAGE.
*
*/

#define _GNU_SOURCE
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <syslog.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/prctl.h>
#include <signal.h>
/* sg3 lib */
#include <scsi/sg_lib.h>
#include <scsi/sg_io_linux.h>

#define DEVNAME_SDA                "/dev/sda"
#define DEVNAME_SDB                "/dev/sdb"
#define STATUS_FILE                "/var/run/status-alert"
#define UART2_CMD_LED_HD_OFF       "\x37" /* status led off */
#define UART2_CMD_LED_HD_GS        "\x38" /* status led green */
#define UART2_CMD_LED_HD_AB        "\x3B" /* status led orange blinking */
#define USB_SCRIPT                 "/etc/syno/buttons/usb"
#define POWER_SCRIPT               "/etc/syno/buttons/power"
#define PM_STATUS_ACTIVE       1
#define PM_STATUS_IDLE         2
#define PM_STATUS_STANDBY      3
#define PM_STATUS_UNKNOWN     -1

#define SAT_ATA_PASS_THROUGH16 0x85
#define SAT_ATA_PASS_THROUGH16_LEN 16
#define SAT_ATA_RETURN_DESC 9  /* ATA Return (sense) Descriptor */
#define ASCQ_ATA_PT_INFO_AVAILABLE 0x1d

#define ATA_CHECK_POWER_MODE 0xe5

#define EBUFF_SZ 256

static int oldstatus = -1;

int getPowerMode(char * file_name, int verbose) {
	char ebuff[EBUFF_SZ];
	int extend = 0, sg_fd;
	int chk_cond = 1;   /* set to 1 to read register(s) back */
	int protocol = 3;   /* non-dat data-in */
	int t_dir = 1;      /* 0 -> to device, 1 -> from device */
	int byte_block = 1; /* 0 -> bytes, 1 -> 512 byte blocks */
	int t_length = 0;   /* 0 -> no data transferred, 2 -> sector count */
	unsigned char sense_buffer[64];
	int k;
	const unsigned char * ucp = NULL;
	
	unsigned char aptCmdBlk[SAT_ATA_PASS_THROUGH16_LEN] =
	{SAT_ATA_PASS_THROUGH16, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0};
	sg_io_hdr_t io_hdr;
	
	if ((sg_fd = open(file_name, O_RDWR)) < 0) {
		snprintf(ebuff, EBUFF_SZ,
			"sg_sat_chk_power: error opening file: %s", file_name);
		perror(ebuff);
		return PM_STATUS_UNKNOWN;
	}
	
	/* Prepare ATA PASS-THROUGH COMMAND (16) command */
	aptCmdBlk[14] = ATA_CHECK_POWER_MODE;
	aptCmdBlk[1] = (protocol << 1) | extend;
	aptCmdBlk[2] = (chk_cond << 5) | (t_dir << 3) |
	(byte_block << 2) | t_length;
	if (verbose) {
		fprintf(stderr, "    ata pass through(16) cdb: ");
		for (k = 0; k < SAT_ATA_PASS_THROUGH16_LEN; ++k)
			fprintf(stderr, "%02x ", aptCmdBlk[k]);
		fprintf(stderr, "\n");
	}
	
	memset(&io_hdr, 0, sizeof(sg_io_hdr_t));
	io_hdr.interface_id = 'S';
	io_hdr.cmd_len = sizeof(aptCmdBlk);
	io_hdr.mx_sb_len = sizeof(sense_buffer);
	io_hdr.dxfer_direction = SG_DXFER_NONE;
	io_hdr.dxfer_len = 0;
	io_hdr.dxferp = NULL;
	io_hdr.cmdp = aptCmdBlk;
	io_hdr.sbp = sense_buffer;
	io_hdr.timeout = 20000;     /* 20000 millisecs == 20 seconds */
	
	if (ioctl(sg_fd, SG_IO, &io_hdr) < 0) {
		perror("sg_sat_chk_power: SG_IO ioctl error");
		close(sg_fd);
		return 1;
	}
	
	/* error processing: N.B. expect check condition, no sense ... !! */
	switch (sg_err_category3(&io_hdr)) {
	case SG_LIB_CAT_CLEAN:
		break;
	case SG_LIB_CAT_RECOVERED:  /* sat-r09 (latest) uses this sk */
	case SG_LIB_CAT_NO_SENSE:   /* earlier SAT drafts used this */
		/* XXX: Until the spec decides which one to go with. 20060607 */
		ucp = sg_scsi_sense_desc_find(sense_buffer, sizeof(sense_buffer),
			SAT_ATA_RETURN_DESC);
		if (NULL == ucp) {
			if (verbose > 1)
			printf("ATA Return Descriptor expected in sense but not "
				"found\n");
			sg_chk_n_print3("ATA_16 command error", &io_hdr, 1);
		} else if (verbose)
		sg_chk_n_print3("ATA Return Descriptor, as expected",
			&io_hdr, 1);
		if (ucp && ucp[3]) {
			if (ucp[3] & 0x4)
				printf("error in returned FIS: aborted command\n");
			else
				printf("error=0x%x, status=0x%x\n", ucp[3], ucp[13]);
		}
		break;
	default:
		fprintf(stderr, "unexpected SCSI sense category\n");
		ucp = sg_scsi_sense_desc_find(sense_buffer, sizeof(sense_buffer),
			SAT_ATA_RETURN_DESC);
		if (NULL == ucp)
			sg_chk_n_print3("ATA_16 command error", &io_hdr, 1);
		else if (verbose)
		sg_chk_n_print3("ATA Return Descriptor, as expected",
			&io_hdr, 1);
		if (ucp && ucp[3]) {
			if (ucp[3] & 0x4)
				printf("error in returned FIS: aborted command\n");
			else
				printf("error=0x%x, status=0x%x\n", ucp[3], ucp[13]);
		}
		break;
	}
	int ret = PM_STATUS_UNKNOWN;
	if (ucp) {
		switch (ucp[5]) {       /* sector_count (7:0) */
		case 0xff:
			if(verbose)
				printf("In active mode or idle mode\n");
			ret = PM_STATUS_ACTIVE;
			break;
		case 0x80:
			if(verbose)
				printf("In idle mode\n");
			ret = PM_STATUS_IDLE;
			break;
		case 0x41:
			if(verbose)
				printf("In NV power mode and spindle is spun or spinning up\n");
			ret = PM_STATUS_UNKNOWN;
			break;
		case 0x40:
			if(verbose)
				printf("In NV power mode and spindle is spun or spinning down\n");
			ret = PM_STATUS_UNKNOWN;
			break;
		case 0x0:
			if(verbose)
				printf("In standby mode\n");
			ret = PM_STATUS_STANDBY;
			break;
		default:
			printf("unknown power mode (sector count) value=0x%x\n", ucp[5]);
			break;
		}
	} 
	close(sg_fd);
	return ret;    
}

int file_exists(char *filename)
{
  struct stat   buffer;   
  return (stat (filename, &buffer) == 0);
}

void setStatusLed(int status, int verbose){
	int ttys1_fd;
	const char const * statusn[]={"standby","active", "alert"};
	
	if(oldstatus == status)
		return; /* status is already set */
	syslog(LOG_INFO, "Device state changed to %s", 
		statusn[status]);
	if(verbose)
		printf("Set status: %d\n", status);
	if ((ttys1_fd = open("/dev/ttyS1", O_WRONLY | O_CLOEXEC)) < 0) {
		fprintf(stderr, "Unable to open /dev/ttyS1\n");
		exit(1);
	}
	switch (status) {
	case 0:
		write(ttys1_fd, UART2_CMD_LED_HD_OFF, 1);
		break;
	case 1:
		write(ttys1_fd, UART2_CMD_LED_HD_GS, 1);
		break;
	case 2:
		write(ttys1_fd, UART2_CMD_LED_HD_AB, 1);
		break;
	}
	close(ttys1_fd);
	oldstatus = status;
}

int main(int argc, char * argv[])
{
	int k;
	char * file_name = 0;
	int verbose = 0;
	
	
	for (k = 1; k < argc; ++k) {
		if (0 == strcmp(argv[k], "-v"))
			++verbose;
		else if (0 == strcmp(argv[k], "-vv"))
			verbose += 2;
		else if (0 == strcmp(argv[k], "-vvv"))
			verbose += 3;
		else if (*argv[k] == '-') {
			printf("Unrecognized switch: %s\n", argv[k]);
			file_name = 0;
			break;
		}
	}
	if(!verbose)
		daemon(0, 0); /*  run in the background */
	/* start button thread */
	pthread_t button_thread;
	void *ButtonControl();
	pthread_create(&button_thread,NULL, ButtonControl, NULL);
	
        for (;;) {
        	if (file_exists(STATUS_FILE))
        		setStatusLed(2, verbose);
        	else {
        		int st_sda = getPowerMode(DEVNAME_SDA, verbose);
        		int st_sdb = getPowerMode(DEVNAME_SDB, verbose);
        		if(st_sda == PM_STATUS_STANDBY && st_sdb == PM_STATUS_STANDBY) {
        			setStatusLed(0, verbose);
        		}
        		else {
        			setStatusLed(1, verbose);
        		}
        	}
        	sleep(1);
        }
        return 0;
}

void *ButtonControl(void){
	int            fd;
	struct termios options;
	/* open the port */
	fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY | O_NDELAY | O_CLOEXEC);
	fcntl(fd, F_SETFL, 0);
	
	/* get the current options */
	tcgetattr(fd, &options);
	
	/* set raw input, 1 second timeout */
	options.c_cflag     |= (CLOCAL | CREAD);
	options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag     &= ~OPOST;
	options.c_cc[VMIN]  = 10;
	options.c_cc[VTIME] = 100;
	/* set speed */
	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);
	/* set the options */
	tcsetattr(fd, TCSANOW, &options);
	char buf[2]={0};
	
	char* argv_usb[] = { USB_SCRIPT, NULL };
	char* argv_power[] = { POWER_SCRIPT, NULL };
	signal(SIGCHLD, SIG_IGN);
	char name [17];	/* Name must be <= 16 characters + a null */
	
	strcpy (name, "button_control");
	prctl (PR_SET_NAME, (unsigned long)&name);

	
	for (;;){
		read(fd,&buf,1);
		int child_pid = fork();
		if(child_pid == 0) { /* child */
			close(STDIN_FILENO);
			close(STDOUT_FILENO);
			close(STDERR_FILENO);
			switch(buf[0]){
			case 0x60:
				syslog(LOG_INFO, "USB button pressed, running %s",
					USB_SCRIPT);
				execve(argv_usb[0], argv_usb, environ);
				break;
			case 0x30:
				syslog(LOG_INFO, "Power button pressed, running %s",
					POWER_SCRIPT);
				execve(argv_power[0], argv_power, environ);
				break;
			default:
				syslog(LOG_INFO, "Unknown even on serial port: 0x%x", buf[0]);
			}
			exit(0); /* end forked child */
		}
		buf[0]=0;
	}
	close(fd);
}
