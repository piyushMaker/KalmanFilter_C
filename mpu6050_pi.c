#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define MPU6050_I2C_ADDRESS 0x68  // MPU6050 I2C address
#define ACCEL_XOUT_H 0x3B  // register address for accelerometer x-axis data (high byte)

int readMPU6050(int* x, int* y, int* z) {
  int i2c_fd;
  uint8_t buffer[6];

  // open I2C device
  i2c_fd = open("/dev/i2c-1", O_RDWR);
  if (i2c_fd < 0) {
    perror("Error opening I2C device\n");
    return 1;
  }

  // set slave address
  if (ioctl(i2c_fd, I2C_SLAVE, MPU6050_I2C_ADDRESS) < 0) {
    perror("Error setting slave address\n");
    return 1;
  }

  // read accelerometer data
  if (read(i2c_fd, buffer, 6) != 6) {
    perror("Error reading accelerometer data\n");
    return 1;
  }

  // close I2C device
  close(i2c_fd);

  // convert data to 16-bit signed integers (two's complement)
  *x = ((int16_t)(buffer[0] << 8 | buffer[1])) / 16384.0;
  *y = ((int16_t)(buffer[2] << 8 | buffer[3])) / 16384.0;
  *z = ((int16_t)(buffer[4] << 8 | buffer[5])) / 16384.0;

  return 0;
}
