

#include "mpu9150.h"

static double acc_sensitivity = ACC_SENSITIVITY / 8.0;   /* default value 2048 LSB/g for full-range 16g */
static double gyro_sensitivity = GYRO_SENSITIVITY / 8.0; /* default value 16.4 LSB per °/s for full range 200°/s */

static uint8_t slave_address = MPU9150_ADDRESS;

/* variables for gyro angle calculation */
static uint8_t gyroinitangle_0 = 0;
static uint8_t gyroinitangle_1 = 0;

static xyz initangle;
static xyz gyro_angles_0;
static xyz gyro_angles_1;

/*=====================================*/

/**
 * \fn
 * \brief
 * \param ado (0 or 1) select MPU9150 slave address
 * */

void mpu9150_setup(uint8_t Sensor, mpu9150_bandwidth_dlpf dlpf)
{
	uint8_t slave_address;

	volatile uint8_t buffer[2];
	/* external interrupt enable */

	DDRD &= 0 << PD2; // PD2 as input
	DDRD &= 0 << PD3; // PD3 as input
	//EIMSK= (1 << INT0) | (1 << INT1);         //Enable INT1/0
	//EICRA = 1<<ISC01 | 1 <<ISC00 |  1<<ISC11 | 1 <<ISC10;  //Set interrupt on rising edge

	twi_master_setup(); /* i2c to master mode */

	slave_address = MPU9150_ADDRESS + Sensor;

	/* reset entire device */
	buffer[0] = PWR_MGMT_1;
	buffer[1] = 0x80;
	twi_write_bytes(slave_address, 2, buffer);
	_delay_ms(100);

	/* out of sleep mode to start the sensor and set clock gyro */
	buffer[0] = PWR_MGMT_1;
	buffer[1] = 0x01;
	twi_write_bytes(slave_address, 2, buffer);

	/* configure digital low-pass filter (dlpf) bandwidth */
	buffer[0] = CONFIG;
	buffer[1] = dlpf;
	twi_write_bytes(slave_address, 2, buffer);

	/* enable data ready interrupt for gyroscope */
	buffer[0] = MPU9150_RA_INT_PIN_CFG;
	buffer[1] = 0x10;
	twi_write_bytes(slave_address, 2, buffer);

	buffer[0] = MPU9150_RA_INT_ENABLE;
	buffer[1] = 0x01;
	twi_write_bytes(slave_address, 2, buffer);

	/* enable FIFO buffer */
	/*buffer[0] = USER_CTRL;
   buffer[1] = 0x40;
   twi_write_bytes (slave_address, 2, buffer);

   _delay_ms (1);

   buffer[0] = FIFO_EN;
   buffer[1] = 0x08;
   twi_write_bytes (slave_address, 2, buffer);*/
}

/**
 * \fn void mpu9150_acc_setup (mpu9150_acc_measurement_range mr)
 * \brief setup accelerometer
 * \param mr measurement range (2, 4, 8 or 16g)
 * */
void mpu9150_acc_setup(mpu9150_acc_measurement_range mr, uint8_t Sensor)
{
	uint8_t slave_address;
	volatile uint8_t buffer[2];
	slave_address = MPU9150_ADDRESS + Sensor;
	/* Accelerometer configuration : full-scale range */
	buffer[0] = ACCEL_CONFIG;
	buffer[1] = (mr << 3);
	twi_write_bytes(slave_address, 2, buffer);

	acc_sensitivity = ACC_SENSITIVITY / (1 << mr); /* LSB/g */
}

/**
 * \fn void mpu9150_gyro_setup (mpu9150_gyro_measurement_range mr)
 * \brief setup gyroscope
 * \param mr measurement range (250, 500, 1000 or 2000°/s)
 * */
void mpu9150_gyro_setup(mpu9150_gyro_measurement_range mr, uint8_t Sensor)
{
	uint8_t slave_address;
	volatile uint8_t buffer[2];
	slave_address = MPU9150_ADDRESS + Sensor;
	/* Gyroscope configuration - full-scale range °/sec */
	buffer[0] = GYRO_CONFIG;
	buffer[1] = (mr << 3);
	twi_write_bytes(slave_address, 2, buffer);

	gyro_sensitivity = GYRO_SENSITIVITY; /// (1 << mr); /* LSB per °/sec */

	_delay_ms(1);
}

/*
 * \fn void mpu9150_compass_setup (void)
 * \brief setup compass, enable i2c by-pass
 * */
void mpu9150_compass_setup_bypass(void) // Pass through mode, MCU directly accesses Ak8975c, doens't work with my sensor, possible hardware fault
{
	volatile uint8_t buffer[2];

	/* compass configuration : bypass mode */
	/* the host application processor will be able to directly access the auxiliary i2c bus */
	buffer[0] = MPU9150_RA_INT_PIN_CFG;
	buffer[1] = I2C_BYPASS_EN;
	twi_write_bytes(slave_address, 2, buffer);
	_delay_ms(1);

	buffer[0] = USER_CTRL;
	buffer[1] = 0x00; //  TWI master status disable(I2C_MST_EN=0)
	twi_write_bytes(slave_address, 2, buffer);

	uint8_t ID = ak8975c_read_device_id();
	serialWrite("compass ID: ");
	serialWrite_int(ID);
	serialWrite("\n\r");
}

void mpu_9150_compass_setup_slave(void) //Slave mode, mpu-9150 copies compass readings to its external registers
{
	volatile uint8_t buffer[2];

	ak8975c_self_test();

	buffer[0] = MPU9150_RA_I2C_SLV0_ADDR;
	buffer[1] = AK8975C_ADDRESS;
	twi_write_bytes(slave_address, 2, buffer); //Set Ak8975c address on Slave0

	buffer[0] = MPU9150_RA_I2C_SLV0_REG;
	buffer[1] = 0x02;
	twi_write_bytes(slave_address, 2, buffer); //Set where reading at slave 0 starts.

	buffer[0] = MPU9150_RA_I2C_SLV0_CTRL;
	buffer[1] = 0x88;
	twi_write_bytes(slave_address, 2, buffer); //Set offset at start reading and enable.

	buffer[0] = MPU9150_RA_I2C_SLV1_ADDR;
	buffer[1] = AK8975C_ADDRESS;
	twi_write_bytes(slave_address, 2, buffer); //Set Ak8975c address on Slave1.

	buffer[0] = MPU9150_RA_I2C_SLV1_REG;
	buffer[1] = 0x0A;
	twi_write_bytes(slave_address, 2, buffer); //Set where reading at slave 1 starts.

	buffer[0] = MPU9150_RA_I2C_SLV1_CTRL;
	buffer[1] = 0x81;
	twi_write_bytes(slave_address, 2, buffer); //Enable at set length to 1.

	buffer[0] = MPU9150_RA_I2C_SLV1_DO;
	buffer[1] = 0x01;
	twi_write_bytes(slave_address, 2, buffer); //override register.

	buffer[0] = MPU9150_RA_I2C_MST_DELAY_CTRL;
	buffer[1] = 0x03;
	twi_write_bytes(slave_address, 2, buffer); //Set delay rate.

	buffer[0] = 0x01;
	buffer[1] = 0x80;
	twi_write_bytes(slave_address, 2, buffer); //Unkown DMP register...... Command found on expample code with no comments...I don't really think it does something....

	buffer[0] = MPU9150_RA_I2C_SLV4_CTRL;
	buffer[1] = 0x04;
	twi_write_bytes(slave_address, 2, buffer); //Set i2c slv4 delay.

	buffer[0] = MPU9150_RA_I2C_SLV1_DO;
	buffer[1] = 0x00;
	twi_write_bytes(slave_address, 2, buffer); //override register.

	buffer[0] = USER_CTRL;
	buffer[1] = 0x20;
	twi_write_bytes(slave_address, 2, buffer); //Enable I2C master mode.

	buffer[0] = MPU9150_RA_I2C_SLV4_CTRL;
	buffer[1] = 0x13;
	twi_write_bytes(slave_address, 2, buffer); //Disable slave 4.
}
/*
 * \fn uint8_t mpu9150_read_device_id (void)
 * \brief read device id
 * \return device id
 * */
uint8_t
mpu9150_read_device_id(uint8_t Sensor)
{
	uint8_t slave_address;
	volatile uint8_t device_id;
	volatile uint8_t reg = WHO_AM_I;
	slave_address = MPU9150_ADDRESS + Sensor;
	twi_read_bytes(slave_address, &reg, 1, &device_id);

	return device_id;
}

/*
 * \fn int16_t mpu9150_read_temperature (void)
 * \return temperature in degrees C = (TEMP_OUT signed / 340) + 35
 * */
int16_t
mpu9150_read_temperature(uint8_t Sensor)
{
	uint8_t slave_address;
	volatile uint8_t buffer[2];
	volatile uint8_t reg = TEMP_OUT_H;
	slave_address = MPU9150_ADDRESS + Sensor;

	twi_read_bytes(slave_address, &reg, 2, buffer);

	int16_t temperature;
	temperature = ((buffer[0] << 8) + buffer[1]);
	temperature /= 340;
	temperature += 35;
	return temperature;
}

/**
 * \fn xyz mpu9150_read_acc_xyz (void)
 * \brief read x-, y-, z-axis acceleration
 * \return x, y and z-axis acceleration value in g
 * */
xyz mpu9150_read_acc_xyz_0(void)
{
	volatile uint8_t reg;
	volatile uint8_t buffer[6];
	xyz acc;
	slave_address = MPU9150_ADDRESS;

	reg = ACCEL_XOUT_H;

	twi_read_bytes(slave_address, &reg, 6, buffer);

	acc.x = (float)(((buffer[1] + (buffer[0] << 8)) / acc_sensitivity) - 0.03019);
	acc.y = (float)(((buffer[3] + (buffer[2] << 8)) / acc_sensitivity) + 0.00961);
	acc.z = (float)(buffer[5] + (buffer[4] << 8)) / acc_sensitivity + 0.14;

	return acc;
}
xyz mpu9150_read_acc_xyz_1(void)
{
	volatile uint8_t reg;
	volatile uint8_t buffer[6];
	xyz acc;

	slave_address = 0x69;

	reg = ACCEL_XOUT_H;
	twi_read_bytes(slave_address, &reg, 6, buffer);

	acc.x = (float)(buffer[1] + (buffer[0] << 8)) / acc_sensitivity;
	acc.y = (float)(buffer[3] + (buffer[2] << 8)) / acc_sensitivity;
	acc.z = (float)(buffer[5] + (buffer[4] << 8)) / acc_sensitivity;

	return acc;
}

/**
 * \fn xyz mpu9150_read_gyro_xyz (void)
 * \brief read x-, y-, z-axis angular speed
 * \return x, y and z-axis angular speed value in °/sec */
xyz mpu9150_read_gyro_xyz_0(void)
{
	volatile uint8_t reg = GYRO_XOUT_H;
	volatile uint8_t buffer[6];
	xyz gyro;
	slave_address = MPU9150_ADDRESS;
	twi_read_bytes(slave_address, &reg, 6, buffer);

	gyro.x = (float)((((buffer[0] << 8) + buffer[1]) / gyro_sensitivity) + 0.8409);
	gyro.y = (float)((((buffer[2] << 8) + buffer[3]) / gyro_sensitivity) - 0.94739);
	// gyro.z = (double) ((((buffer[4] << 8) + buffer[5]) / gyro_sensitivity)+0.12623);

	return gyro;
}

xyz mpu9150_read_gyro_xyz_1(void)
{
	volatile uint8_t reg = GYRO_XOUT_H;
	volatile uint8_t buffer[6];
	xyz gyro;
	slave_address = 0x69;
	twi_read_bytes(slave_address, &reg, 6, buffer);

	gyro.x = (float)((((buffer[0] << 8) + buffer[1]) / gyro_sensitivity) + 0.29972);
	gyro.y = (float)((((buffer[2] << 8) + buffer[3]) / gyro_sensitivity) + 3.95968);
	// gyro.z = (double) ((((buffer[4] << 8) + buffer[5]) / gyro_sensitivity));

	return gyro;
}

xyz mpu9150_read_compass_xyz_slave(uint8_t Sensor)
{
	volatile uint8_t reg = MPU9150_RA_EXT_SENS_DATA_01;
	volatile uint8_t buffer[6];
	xyz compass;
	slave_address = MPU9150_ADDRESS + Sensor;
	twi_read_bytes(slave_address, &reg, 6, buffer);

	compass.x = (double)(buffer[0] + (buffer[1] << 8)); //* 0.3; // 0.3µT/LSB (Compass sensitivity)
	compass.y = (double)(buffer[2] + (buffer[3] << 8)) * 0.3;
	//	 compass.z = (double) (buffer[4] + (buffer[5] << 8)) * 0.3;

	return compass;
}

/**
 * \fn xyz mpu9150_read_gyro_xyz (void)
 * \brief read x, y and z-axis magnetic field
 * \return x, y and z-axis magnetic field value in µT */

/*
xyz mpu9150_get_compass_heading (void)
{
  //offset algorythm to be written
  
 xyz acc= mpu9150_read_acc_xyz_0 ();
 
 
 xyz compass=ak8975c_read_xyz();
 
 double hdg_flat = compass.y; //atan2(compass.x,compass.y);
 double hdg_x=compass.x*(1-acc.x*acc.x)-compass.y*acc.x*acc.y-compass.z*acc.x*sqrt(1-acc.x*acc.x-acc.y*acc.y);
 double hdg_y=compass.y*sqrt(1-acc.x*acc.x-acc.y*acc.y)-compass.z*acc.y;
 return hdg_flat; 
if (hdg_x>0)
	{
	double heading= 180-atan2(hdg_y,hdg_x);
	return heading;
	}
if ((hdg_x>0)&& (hdg_y<0))
	{
	double heading=	-atan2(hdg_y,hdg_x);
	return heading;
	}
if ((hdg_x>0) && (hdg_y<0))
	{
	 double heading = 360-atan2(hdg_y,hdg_x);
	 return heading;
	}
if ((hdg_x=0) && (hdg_y<0))
{
	double heading= 90;
    return heading;
}
if ((hdg_x=0) && (hdg_y>0))
	{
	double heading=270;
	}
 
 return compass;
}
*/

xyz mpu9150_get_acc_angles_0()
{
	xyz acc = mpu9150_read_acc_xyz_0();
	xyz angle;

	angle.x = (180 / pi) * (Rajan_FastArcTan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z))); //roll

	angle.y = (180 / pi) * (Rajan_FastArcTan2(sqrt(acc.x * acc.x + acc.z * acc.z), acc.y)) - 90; //pitch

	// angle.z= (180/pi)*(Rajan_FastArcTan2(sqrt(acc.x*acc.x+acc.y*acc.y),acc.z)); //yaw
	return angle;
}

xyz mpu9150_get_acc_angles_1()
{
	xyz acc = mpu9150_read_acc_xyz_1();
	xyz angle;

	angle.x = (180 / pi) * (Rajan_FastArcTan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z))); //roll

	angle.y = (180 / pi) * (Rajan_FastArcTan2(sqrt(acc.x * acc.x + acc.z * acc.z), acc.y)) - 90; //pitch

	//	angle.z= (180/pi)*(Rajan_FastArcTan2(sqrt(acc.x*acc.x+acc.y*acc.y),acc.z)); //yaw
	return angle;
}

xyz mpu9150_get_gyro_angles_0(xyz oldangle)
{
	xyz newangle;
	//Set the angle once
	if (gyroinitangle_0 == 0)
	{
		initangle.x = 0;
		initangle.y = 0;
		initangle.z = 0;

		for (uint8_t i = 0; i < 100; i++)
		{
			oldangle.x += mpu9150_get_acc_angles_0().y;
			oldangle.y += mpu9150_get_acc_angles_0().x;
			//oldangle.z+=mpu9150_get_acc_angles_0().z;
			_delay_ms(1);
		}
		oldangle.x /= 100;
		oldangle.y /= 100;
		//oldangle.z/=100;
		gyroinitangle_0 = 1;
	}
	else
	{
		initangle = mpu9150_read_gyro_xyz_0();
	}

	newangle.x = (initangle.x * 0.002) * 2 + oldangle.x;
	newangle.y = (initangle.y * 0.002) * 2 + oldangle.y;
	//newangle.z=(initangle.z*0.002)*2+oldangle.z;

	return newangle;
}

xyz mpu9150_get_gyro_angles_1(xyz oldangle)
{
	xyz newangle;
	//Set the angle once
	if (gyroinitangle_1 == 0)
	{
		initangle.x = 0;
		initangle.y = 0;
		initangle.z = 0;

		oldangle.x = mpu9150_get_acc_angles_1().y;
		oldangle.y = mpu9150_get_acc_angles_1().x;
		oldangle.z = mpu9150_get_acc_angles_1().z;

		gyroinitangle_1 = 1;
	}

	else
	{
		initangle = mpu9150_read_gyro_xyz_1();
	}

	newangle.x = (initangle.x * 0.002) * 2 + oldangle.x;
	newangle.y = (initangle.y * 0.002) * 2 + oldangle.y;
	//newangle.z=initangle.z*0.002)*2+oldangle.z;

	return newangle;
}

uint8_t mpu9150_read_int_status(uint8_t Sensor)
{
	slave_address = MPU9150_ADDRESS + Sensor;
	volatile uint8_t status;
	volatile uint8_t reg = MPU9150_RA_INT_PIN_CFG;
	twi_read_bytes(slave_address, &reg, 1, &status);
	return status;
}

xyz mpu9150_get_complement_angles_0(uint8_t filter_gain)

{
	xyz comp_angles;

	gyro_angles_0 = mpu9150_get_gyro_angles_0(gyro_angles_0);
	xyz acc_angles = mpu9150_get_acc_angles_0();
	comp_angles.x = filter_gain * gyro_angles_0.y + (1 - filter_gain) * acc_angles.x;
	comp_angles.y = filter_gain * gyro_angles_0.x + (1 - filter_gain) * acc_angles.y;
	//comp_angles.z= filter_gain*gyro_angles_0.z+(1-filter_gain)*acc_angles.z;
	return comp_angles;
}

xyz mpu9150_get_complement_angles_1(uint8_t filter_gain)

{
	xyz comp_angles;
	gyro_angles_1 = mpu9150_get_gyro_angles_1(gyro_angles_1);
	xyz acc_angles = mpu9150_get_acc_angles_1();
	comp_angles.x = filter_gain * gyro_angles_1.y + (1 - filter_gain) * acc_angles.x;
	comp_angles.y = filter_gain * gyro_angles_1.x + (1 - filter_gain) * acc_angles.y;
	return comp_angles;

	return comp_angles;
}

inline float Rajan_FastArcTan(float x)
{
	return 3.1415926 / 4.0 * x - x * (fabs(x) - 1) * (0.2447 + 0.0663 * fabs(x));
}

inline float Rajan_FastArcTan2(float y, float x)
{

	uint8_t qCode;
	const float pi_2 = 3.1415926 / 2.0;
	float q;
	float z;

	// 6 us
	uint8_t swap45 = (fabs(y) > fabs(x));

	// 22us
	if ((y >= 0) && (x >= 0))
	{
		qCode = 0;
	}
	if ((y >= 0) && (x <= 0))
	{
		qCode = 1;
	}
	if ((y <= 0) && (x <= 0))
	{
		qCode = 2;
	}
	if ((y <= 0) && (x >= 0))
	{
		qCode = 3;
	}

	// 54 us
	if (swap45)
	{
		q = x / y;
	}
	else
	{
		q = y / x;
	}

	// 92 us
	z = Rajan_FastArcTan(q);

	if (swap45)
	{
		switch (qCode)
		{
		case 0:
			z = pi_2 - z;
			break;
		case 1:
			z = pi_2 - z;
			break;
		case 2:
			z = -pi_2 - z;
			break;
		case 3:
			z = -pi_2 - z;
			break;
		}
	}
	else
	{
		switch (qCode)
		{
		case 0:
			z = z;
			break;
		case 1:
			z = 3.1415926 + z;
			break;
		case 2:
			z = -3.1415926 + z;
			break;
		case 3:
			z = z;
			break;
		}
	}

	return z;
}