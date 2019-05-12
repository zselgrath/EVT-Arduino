#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

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


	fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY);
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

	//options.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode...
	//options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	// 8N1
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	// no flow control
	options.c_cflag &= ~CRTSCTS;

	//toptions.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset

	options.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
	options.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
	options.c_oflag &= ~OPOST; // make raw	

	tcsetattr(fd, TCSANOW, &options); //Set the new options for the port...

	return (fd);
}

ssize_t serialread(int fd, void *buf, size_t count) {
	// Initialize file descriptor sets
	fd_set read_fds, write_fds, except_fds;
	FD_ZERO(&read_fds);
	FD_ZERO(&write_fds);
	FD_ZERO(&except_fds);
	FD_SET(fd, &read_fds);

	// Set timeout to 1.0 seconds
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0;

	// Wait for input to become ready or until the time out; the first parameter is
	// 1 more than the largest file descriptor in any of the sets
	if (select(fd + 1, &read_fds, &write_fds, &except_fds, &timeout) == 1) {
		// fd is ready for reading
		return read(fd, buf, count);
	}
	else {
		// timeout or error
		std::cout << "timeout" << std::endl;
		return -1;
	}
}

int main () {
	int fd = open_port();

	unsigned char buf;

	unsigned char count = 0;

	sleep(2);
	for(int i = 0; i < 127; ) {

		write(fd, &count, 1);
		if(serialread(fd, &buf, 1) != -1) {
			std::cout << (int)buf << " " << std::flush;
			if(buf != count) std::cout << "oops" << std::endl;
		}
		count++;

	}
}
