/*
 * ak8975c.h
 *
 *  Created on: 24 oct. 2014
 *      Author: ldo
 */

#ifndef AK8975C_H_
#define AK8975C_H_

#include "twi.h"
#include "imu.h"

#define AK8975C_ADDRESS 0x8C

/* Register Map of Compass AK8975C */
#define WIA 0x00
#define INFO 0x01
#define STATUS_1 0x02
#define COMPASS_XOUT_L 0x03
#define COMPASS_XOUT_H 0x04
#define COMPASS_YOUT_L 0x05
#define COMPASS_YOUT_H 0x06
#define COMPASS_ZOUT_L 0x07
#define COMPASS_ZOUT_H 0x08
#define STATUS_2 0x09
#define CNTL 0x0A
#define ASTC 0x0C

/*compass configuration */
#define COMPASS_POWER_DOWN 0x00
#define COMPASS_SINGLE_MEASUREMENT 0x01
#define COMPASS_SELF_TEST 0x08
#define FUSE_ROM_ACCESS_MODE 0x0F

#define GENERATE_FIELD 0x40

#define COMPASS_SENSITIVITY 0.3 // 0.3µT/LSB

void ak8975c_setup(void);

uint8_t
ak8975c_read_device_id(void);

xyz ak8975c_self_test(void);

xyz ak8975c_read_xyz(void);

xyz ak8975c_fuze_mode_read(void);
#endif /* AK8975C_H_ */
