#define IOCTL_READ_LINE 0x08
#define IOCTL_READ_MULTIPLE_LINES 0x09
#define IOCTL_WRITE_LINE 0x0A
#define IOCTL_WRITE_MULTIPLE_LINES 0x0B
#define IOCTL_DIRECTION_OUTPUT 0x0C
#define IOCTL_DIRECTION_INPUT 0x0D
#define IOCTL_REQUEST_GPIO 0x0E
#define IOCTL_FREE_GPIO 0x0F
#define IOCTL_REQUEST_IRQ_GPIO 0x10
#define IOCTL_FREE_IRQ_GPIO 0x20
#define MAX_GPIO_OFFSET 40

struct request
{
	int pinNumber[MAX_GPIO_OFFSET];
	int val[MAX_GPIO_OFFSET];
	int numOfPins;
};
