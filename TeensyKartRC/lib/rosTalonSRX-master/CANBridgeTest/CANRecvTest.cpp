#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <unistd.h> //for sleep

#include <iostream>

#define BAUD B115200
#define PORT "/dev/ttyACM0"

/*
 * 'open_port()' - Open serial port 1.
 *
 * Returns the file descriptor on success or -1 on error.
 */

int open_port(void) {
	int fd; /* File descriptor for the port */


	fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY| O_NONBLOCK);
	if (fd == -1)
	{
		/*
		* Could not open the port.
		*/

		perror("open_port: Unable to open /dev/ttyf1 - ");
	}
	else fcntl(fd, F_SETFL, 0);

	struct termios options;

	tcgetattr(fd, &options); //Get the current options for the port...

	cfsetispeed(&options, BAUD); //Set the baud rates
	cfsetospeed(&options, BAUD);

	options.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode...
	//options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	tcsetattr(fd, TCSANOW, &options); //Set the new options for the port...

	return (fd);
}

ssize_t serialread(int fd, void *buf, size_t count, long int timeoutus) {
	// Initialize file descriptor sets
	fd_set read_fds, write_fds, except_fds;
	FD_ZERO(&read_fds);
	FD_ZERO(&write_fds);
	FD_ZERO(&except_fds);
	FD_SET(fd, &read_fds);

	// Set timeout to 1.0 seconds
	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = timeoutus;

	// Wait for input to become ready or until the time out; the first parameter is
	// 1 more than the largest file descriptor in any of the sets
	if (select(fd + 1, &read_fds, &write_fds, &except_fds, &timeout) == 1) {
		// fd is ready for reading
		return read(fd, buf, count);
	}
	else {
		// timeout or error
		std::cout << "timeout " << std::flush;
		return -1;
	}
}

#define DATTIME 10000

int main () {
	int fd = open_port();

	unsigned int buf;
	unsigned char size;
	unsigned char packetcount;
	unsigned char checksum;
	unsigned int arbID;
	unsigned char bytes[8];
	
	sleep(2);

	for(int i = 0; i < 1; i++) {
		//usleep(5000);
		write(fd, "d", 1);
		if(serialread(fd, &size, 1, 15000) != -1 && size <= 8) {  
			if(serialread(fd, &packetcount, 1, DATTIME) != -1) {
				if(serialread(fd, &checksum, 1, DATTIME) != -1 && checksum == 42) {
					if(serialread(fd, &arbID, 4, DATTIME) != -1 && arbID < 536870912) {
						if(serialread(fd, &bytes, size, DATTIME) != -1 ) {
							std::cout << "size:" << unsigned(size) << " pcktcnt:" << unsigned(packetcount) << "\tchksum:" << unsigned(checksum) << "\tarbID:"<< unsigned(arbID) << "\tbytes:";
							for(int j = 0; j < size; j++) {
								std::cout << unsigned(bytes[j]) << " ";
							}
							std::cout << std::endl;
						} else {std::cout << "byte err " << std::endl;}
					} else {std::cout << "arID err " << unsigned(arbID) << std::endl;}
				} else {std::cout << "cksm err " << unsigned(checksum) << std::endl;}
			} else {std::cout << "pcnt err " << std::endl;}
		} else {std::cout << "size err " << unsigned(size) << std::endl;}
	}
}
