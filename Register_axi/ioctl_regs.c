#include <sys/ioctl.h>                                                                           
#include <sys/types.h>                                                                           
#include <sys/stat.h>   
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ioctl_regs_define.h"







int main(int argc, char* argv[])  // " argc" 'e il numero digli argomenti passati, "argv" 'e il vettore che li contiene
																	// NOTA: in argv[0] e' contenuto il nome del programma (ioctl_regs)
{

	int fd;								// File descriptor
	int ioctl_id;
	int ioctl_arg;
	int ioctl_tag;
	int ret;
	
	if (argc != 4) {																			// verifia che il numero di argomenti sia 4 
		fprintf(stderr, "Errato numero di argomenti\n");
			return -3;
		}
	
	else if (strcmp(argv[2], DEST_CLKGEN) == 0) {					// caso del comando "clkgen" 
		ioctl_id = IOCTL_CLKGEN;
		if (strcmp(argv[3], CMD_OFF) == 0)
			ioctl_arg = OFF;
		else if (strcmp(argv[3], CMD_ON) == 0)
			ioctl_arg = ON;	
		else if (strcmp(argv[3], CMD_RESET) == 0)
			ioctl_arg = RESET;		
		else 
		{
			fprintf(stderr, "Comando errato: non e' possibie lo stato '%s' per il Clock Generator\n", argv[3]);
			return -1;
		}
	}
	
	else if (strcmp(argv[2], DEST_LED) == 0) {						// caso del comando "clkgen" 
		ioctl_id = IOCTL_LED;
		if (strcmp(argv[3], CMD_OFF) == 0)
			ioctl_arg = OFF;
		else if (strcmp(argv[3], CMD_ON) == 0)
			ioctl_arg = ON;	
		else 
		{
			fprintf(stderr, "Comando errato: non e' possibie lo stato '%s' per il Led\n", argv[3]);
			return -1;
		}
	}
	
	else
				{
			fprintf(stderr, "Comando sconosciuto\n");
			return -1;
		}

	ioctl_tag = _IOW(MAGIC_NUMBER, ioctl_id, int);				// Combina le informazioni utili ad identificare la ioctl come la sintasi della chiamata si aspetta


	fd = open(argv[1], O_RDWR);														// Apre il file del driver a caratteri; se fallisce, fornisce il valore -1
	if (fd = -1)
		{
			fprintf(stderr, "Impossibile aprire il file %s \n", argv[1]);
			return -2;
		}

	ret = ioctl(fd, ioctl_tag, ioctl_arg);								// chiama la ioctl; se fallisce, fornisce il valore -1
	if (ret = -1)
		{
			fprintf(stderr, "Impossibile eseguire il comando %s \n", argv[2]);
			return -2;
		}


}
