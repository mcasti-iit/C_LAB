#include <sys/ioctl.h>                                                                           
#include <sys/types.h>                                                                           
#include <sys/stat.h>   
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#include "ad9522_define.h"



	
int main(int argc, char* argv[])
{

	int 	ret;
	int   fd;
	int   ioctl_cmd;
	void* ioctl_ptr;
	int   value = 33;
	int   i;
	
	
  #define true 1
	#define false 0
	typedef int bool;
	bool is_read = false;

	fd = open("/dev/iit-AD9522-0", O_RDWR);
	
	if (argc < 2) {
		fprintf(stderr, "Specificare un comando\n");
	  return -1;
	}
	
	for (i=1;i<argc;i++) {                          // Il loop serve per ciclare in cerca di piu' comandi
	
		if (strcmp(argv[i], POWDOWN_ARGV) == 0) {									// Comparazione di stringhe
			ioctl_cmd = _IO(MAGIC_NUMBER, POWDOWN_CMD);
			ioctl_ptr = &value;
			is_read = false;
		}
		
		else if (strcmp(argv[i], POWUP_ARGV) == 0) {							// Comparazione di stringhe
			ioctl_cmd = _IO(MAGIC_NUMBER, POWUP_CMD);
			ioctl_ptr = &value;
			is_read = false;
		}
		
		else if (strcmp(argv[i], RESET_ARGV) == 0) {							// Comparazione di stringhe
			ioctl_cmd = _IO(MAGIC_NUMBER, RESET_CMD);
			ioctl_ptr = &value;
			is_read = false;
		}
		
		else if (strcmp(argv[i], STATUS_ARGV) == 0) {							// Comparazione di stringhe
			ioctl_cmd = _IOR(MAGIC_NUMBER, STATUS_CMD, int*);
			ioctl_ptr = &value;
			is_read = true;
		}
		
		else if (strcmp(argv[i], INIT_ARGV) == 0) {								// Comparazione di stringhe
			ioctl_cmd = _IO(MAGIC_NUMBER, INIT_CMD);
			ioctl_ptr = &value;
			is_read = false;
		}
		
		else if (strcmp(argv[i], ENGTCLK_ARGV) == 0) {						// Comparazione di stringhe
			ioctl_cmd = _IO(MAGIC_NUMBER, ENGTCLK_CMD);
			ioctl_ptr = &value;
			is_read = false;
		}	
		
		else if (strcmp(argv[i], ENSPCLK_ARGV) == 0) {						// Comparazione di stringhe
			ioctl_cmd = _IO(MAGIC_NUMBER, ENSPCLK_CMD);
			ioctl_ptr = &value;
			is_read = false;
		}
		
		else { 
			fprintf(stderr, "Comando sconosciuto\n");
//			return -2;
		}


	
	

		// fd = open(argv[1], O_RDWR);

		//ioctl(fd, tmp_ioctl_cmd, &value);
		ret = ioctl(fd, ioctl_cmd, ioctl_ptr);
		if (ret < 0) {																																			// 	Se non va a buon fine
			printf("IOCTL fallita\n");																													// 	- stampa il messaggio di errore 
  	 	return ret;																																					// 	- ed esce
		}
		
		if (is_read)
			printf("AD9522 Status register: 0x%02X\n", value);

	}
	
	  printf("Command executed with exit value: %d\n", ret);
	
																																		
			
}
		
