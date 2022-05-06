#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

int main()
{
	int fd;
	char buf[512];
	
	
	fd = open("/home/icub/regs", O_RDONLY);  // Apre il file ed assegna il file descriptor
	
	read(fd, buf, 5);		// Legge dal file 5 bytes
	buf[5] = '\0'; 			// Per terminare la riga
	printf(buf);
	
	read(fd, buf, 10);	// Legge dal file 5 bytes
	buf[10] = '\0'; 			// Per terminare la riga
	printf(buf);
	
	return 0;
}
