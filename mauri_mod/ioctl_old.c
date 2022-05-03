#include <sys/ioctl.h>                                                                           
#include <sys/types.h>                                                                           
#include <sys/stat.h>   
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>



#define MAGIC_NUMBER 32

#define EXT_CMD_1   	"write"
#define EXT_CMD_2   	"read"
#define EXT_CMD_3   	"mauval"

#define IOCTL_CMD_1   1
#define IOCTL_CMD_2   2
#define IOCTL_CMD_3   3


	typedef struct {
	int variable_a;
	int variable_b;
} maustruct;

	
int main(int argc, char* argv[])
{

	int fd;
	int tmp_ioctl_cmd;
	int value;
	int is_read = 0;
	void* tmp_ptr;
	
	maustruct ioctl_mauval;

	
	if (argc < 2) {
		fprintf(stderr, "Suca\n");
	  return -1;
	}
	
	if (strcmp(argv[1], EXT_CMD_1) == 0) {
		if (argc != 3) {
			fprintf(stderr, "Non so cosa scrivere\n");
			return -3;
		}
		value = atoi(argv[2]);
		tmp_ptr = &value;
		tmp_ioctl_cmd = _IOW(MAGIC_NUMBER, IOCTL_CMD_1, int*);
	}
	
	else if (strcmp(argv[1], EXT_CMD_2) == 0) {
		if (argc != 2) {
			fprintf(stderr, "Parametro inatteso\n");
			return -4;
		}
		tmp_ptr = &value;
		tmp_ioctl_cmd = _IOR(MAGIC_NUMBER, IOCTL_CMD_2, int*);
		is_read = 1;
	}
	
	else if (strcmp(argv[1], EXT_CMD_3) == 0) {
		if (argc != 4) {
			fprintf(stderr, "Numero parametri inatteso\n");
			return -5;
		}
		ioctl_mauval.variable_a = atoi(argv[2]);
		ioctl_mauval.variable_b = atoi(argv[3]);
		tmp_ptr = &ioctl_mauval;
		tmp_ioctl_cmd = _IOW(MAGIC_NUMBER, IOCTL_CMD_3, maustruct*);
	}
	
	else { 
		fprintf(stderr, "Fanculizzati\n");
		return -2;
	}
	
	
	fd = open("/dev/edeye_0", O_RDWR);

	//ioctl(fd, tmp_ioctl_cmd, &value);
	ioctl(fd, tmp_ioctl_cmd, tmp_ptr);
	
	if (is_read == 1)
		printf(" Valore precedentemente scritto: %d\n",value);
	
	
	
	
		
		


}
