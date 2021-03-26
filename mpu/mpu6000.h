//ioctl commands
#define IOCTL_MPU_RESET 0x10
#define IOCTL_MPU_WRITE 0x20
#define IOCTL_MPU_READ 0x30
#define IOCTL_MPU_TEMP 0x80
#define IOCTL_MPU_ACCEL_XOUT 0x90
#define IOCTL_MPU_ACCEL_YOUT 0xA0
#define IOCTL_MPU_ACCEL_ZOUT 0xB0
#define IOCTL_MPU_GYRO_XOUT 0xC0
#define IOCTL_MPU_GYRO_YOUT 0xD0
#define IOCTL_MPU_GYRO_ZOUT 0xE0
#define IOCTL_MPU_ENABLE_FIFO 0xF0
#define IOCTL_MPU_FIFO_LOCK 0x11
#define IOCTL_MPU_FIFO_UNLOCK 0x12

//register addresses
#define ID 0x75
#define FIFO_EN 0x23
#define PWR_MGMT_1 0x6B
#define CONFIG 0x1A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define USER_CONTROL 0x6A
#define FIFO_COUNT_H 0x72
#define FIFO_COUNT_L 0x73
#define FIFO_R_W 0x74

#include <linux/types.h>

typedef __u8 u8;

struct mpu_request {
    u8 address;
    u8 value;
};
