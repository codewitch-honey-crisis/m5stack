/*
MIT License

Copyright (c) 2022 honey the codewitch

Portions copyright (C) 2017 M5 Stack

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <Arduino.h>
#include "sh200q.hpp"
#define sampleFreq	25.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 1.0f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain
//#define twoKiDef	(0.0f * 0.0f)

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

//float invSqrt(float x);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
#pragma GCC diagnostic pop
//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void mahony_ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		//MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = sqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void mahony_ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az,float *pitch,float *roll,float *yaw) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;


	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

    if(pitch!=nullptr) {
	    *pitch = asin(-2 * q1 * q3 + 2 * q0* q2);	// pitch
        *pitch *= RAD_TO_DEG;
    }
    if(roll!=nullptr) {
	    *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1);	// roll
        *roll  *= RAD_TO_DEG;
    }
    if(yaw!=nullptr) {
	    *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);	//yaw
        *yaw   *= RAD_TO_DEG;
        // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
        // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
        // - http://www.ngdc.noaa.gov/geomag-web/#declination
        *yaw   -= 8.5;
    }
}

bool sh200q_i2c_read_bytes(TwoWire& w, uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t count) {
    w.beginTransmission(address);
    w.write(reg);
    uint8_t i = 0;
    if(w.endTransmission(false) == 0) {
        if (w.requestFrom(address, (uint8_t)count)) {
            while (w.available()) {
                Serial.println("read byte");
                buffer[i++] = w.read();
            }
            return true;
        } else {
            Serial.println("request failed");
        }
    } else {
        Serial.println("write failed");
    }
    return false;
}
bool sh200q_i2c_write_bytes(TwoWire& w, uint8_t address, uint8_t reg, uint8_t* data, uint8_t length) {
    w.beginTransmission(address);
    w.write(reg);
    for (int i = 0; i < length; i++) {
        w.write(*(data + i));
    }
    return (w.endTransmission() == 0);  // Send the Tx buffer
}

namespace arduino {
void sh200q::update_gres() {
    switch (m_gyro_scale) {
        case sh200q_gyro_scale::scale_125dps:
            m_gres = 125.0 / 32768.0;
            break;
        case sh200q_gyro_scale::scale_250dps:
            m_gres = 250.0 / 32768.0;
            break;
        case sh200q_gyro_scale::scale_500dps:
            m_gres = 500.0 / 32768.0;
            break;
        case sh200q_gyro_scale::scale_1000dps:
            m_gres = 1000.0 / 32768.0;
            break;
        case sh200q_gyro_scale::scale_2000dps:
            m_gres = 2000.0 / 32768.0;
            break;
    }
}
void sh200q::update_ares() {
    switch (m_acc_scale) {
        case sh200q_acc_scale::scale_4g:
            m_ares = 4.0 / 32768.0;
            break;
        case sh200q_acc_scale::scale_8g:
            m_ares = 8.0 / 32768.0;
            break;
        case sh200q_acc_scale::scale_16g:
            m_ares = 16.0 / 32768.0;
            break;
    }
}
void sh200q::adc_reset() {
    unsigned char tempdata[1];
    sh200q_i2c_read_bytes(m_i2c, address, op_adc_reset, tempdata, 1);

    tempdata[0] = tempdata[0] | 0x04;
    sh200q_i2c_write_bytes(m_i2c, address, op_adc_reset, tempdata, 1);
    delay(1);

    tempdata[0] = tempdata[0] & 0xFB;
    sh200q_i2c_write_bytes(m_i2c, address, op_adc_reset, tempdata, 1);
}
bool sh200q::initialize() {
    if (!m_initialized) {
        unsigned char tempdata[1];
        m_gyro_scale = sh200q_gyro_scale::scale_2000dps;
        m_acc_scale = sh200q_acc_scale::scale_8g;
        m_i2c.begin();
        sh200q_i2c_read_bytes(m_i2c, address, op_whoami, tempdata, 1);
        if (tempdata[0] != 0x18) {
            return false;
        }
        adc_reset();

        sh200q_i2c_read_bytes(m_i2c, address, 0xD8, tempdata, 1);

        tempdata[0] = tempdata[0] | 0x80;
        sh200q_i2c_write_bytes(m_i2c, address, 0xD8, tempdata, 1);

        delay(1);

        tempdata[0] = tempdata[0] & 0x7F;
        sh200q_i2c_write_bytes(m_i2c, address, 0xD8, tempdata, 1);

        tempdata[0] = 0x61;
        sh200q_i2c_write_bytes(m_i2c, address, 0x78, tempdata, 1);

        delay(1);

        tempdata[0] = 0x00;
        sh200q_i2c_write_bytes(m_i2c, address, 0x78, tempdata, 1);

        tempdata[0] = 0x91;  // 0x81 1024hz   //0x89 512hz    //0x91  256hz
        sh200q_i2c_write_bytes(m_i2c, address, op_acc_config, tempdata, 1);

        // set gyro odr 500hz
        tempdata[0] = 0x13;  // 0x11 1000hz    //0x13  500hz   //0x15  256hz
        sh200q_i2c_write_bytes(m_i2c, address, op_gyro_config, tempdata, 1);

        // set gyro dlpf 50hz
        tempdata[0] = 0x03;  // 0x00 250hz   //0x01 200hz   0x02 100hz  0x03 50hz  0x04 25hz
        sh200q_i2c_write_bytes(m_i2c, address, op_gyro_dlpf, tempdata, 1);

        // set no buffer mode
        tempdata[0] = 0x00;
        sh200q_i2c_write_bytes(m_i2c, address, op_fifi_config, tempdata, 1);

        // set acc range +-8G
        tempdata[0] = 0x01;
        sh200q_i2c_write_bytes(m_i2c, address, op_acc_range, tempdata, 1);

        // set gyro range +-2000/s
        tempdata[0] = 0x00;
        sh200q_i2c_write_bytes(m_i2c, address, op_gyro_range, tempdata, 1);

        tempdata[0] = 0xC0;
        sh200q_i2c_write_bytes(m_i2c, address, op_reg_set1, tempdata, 1);

        sh200q_i2c_read_bytes(m_i2c, address, op_reg_set2, tempdata, 1);

        tempdata[0] = tempdata[0] | 0x10;
        sh200q_i2c_write_bytes(m_i2c, address, op_reg_set2, tempdata, 1);
        delay(1);

        tempdata[0] = tempdata[0] & 0xEF;
        sh200q_i2c_write_bytes(m_i2c, address, op_reg_set2, tempdata, 1);
        delay(10);
        m_initialized = true;
        gyro_scale(m_gyro_scale);
        acc_scale(m_acc_scale);
    }
    return true;
}
void sh200q::acc_scale(sh200q_acc_scale value) {
    initialize();
    uint8_t reg;
    sh200q_i2c_read_bytes(m_i2c, address, op_acc_range, &reg, 1);
    reg = (reg & 0xf8) | (((uint8_t)value) & 0x07);
    sh200q_i2c_write_bytes(m_i2c, address, op_acc_range, &reg, 1);
    delay(10);
    m_acc_scale = value;
    update_ares();
}
void sh200q::gyro_scale(sh200q_gyro_scale value) {
    initialize();
    uint8_t reg;
    sh200q_i2c_read_bytes(m_i2c, address, op_gyro_range, &reg, 1);
    reg = (reg & 0xf8) | (((uint8_t)value) & 0x07);
    sh200q_i2c_write_bytes(m_i2c, address, op_gyro_range, &reg, 1);
    delay(10);
    m_gyro_scale = value;
    update_gres();
}
void sh200q::acc(float* out_x, float* out_y, float* out_z) {
    initialize();
    uint8_t buf[6];
    sh200q_i2c_read_bytes(m_i2c, address, op_output_acc, buf, 6);
    if (out_x != nullptr) {
        *out_x = (int16_t)((buf[1] << 8) | buf[0]) * m_ares;
    }
    if (out_y != nullptr) {
        *out_y = (int16_t)((buf[3] << 8) | buf[2]) * m_ares;
    }
    if (out_z != nullptr) {
        *out_z = (int16_t)((buf[5] << 8) | buf[4]) * m_ares;
    }
}
void sh200q::gyro(float* out_x, float* out_y, float* out_z) {
    initialize();
    uint8_t buf[6];
    sh200q_i2c_read_bytes(m_i2c, address, op_output_gyro, buf, 6);
    if (out_x != nullptr) {
        *out_x = (int16_t)((buf[1] << 8) | buf[0]) * m_gres;
    }
    if (out_y != nullptr) {
        *out_y = (int16_t)((buf[3] << 8) | buf[2]) * m_gres;
    }
    if (out_z != nullptr) {
        *out_z = (int16_t)((buf[5] << 8) | buf[4]) * m_gres;
    }
}
float sh200q::temp() {
    initialize();
    uint8_t buf[2];
    sh200q_i2c_read_bytes(m_i2c, address, op_output_temp, buf, 2);
    return (int16_t)((buf[1] << 8) | buf[0]) / 333.87 + 21.0;
}
void sh200q::ahrs(float *pitch, float *roll, float *yaw) {

  float accX = 0; 
  float accY = 0;
  float accZ = 0;

  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;


  gyro(&gyroX, &gyroY, &gyroZ);
  acc(&accX, &accY, &accZ);
  
  mahony_ahrs_update_imu(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ, pitch, roll, yaw);
}
}  // namespace arduino