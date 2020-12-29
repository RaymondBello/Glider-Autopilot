clc; clear;

% IMU CHARACTERISTICS [MPU-9250] %

% Digital-output X-, Y-, and Z-Axis angular rate sensors (gyroscopes) with a 
% user-programmable fullscale range of ±250, ±500, ±1000, and ±2000°/sec 
% and integrated 16-bit ADCs
% FS_SEL=0 131 LSB/(º/s)
% FS_SEL=1 65.5 LSB/(º/s)
% FS_SEL=2 32.8 LSB/(º/s)
% FS_SEL=3 16.4 LSB/(º/s)
GYRO_FSR = 2000;
GYRO_SENS = 16.4;
GYRO_GAIN = (GYRO_SENS/1000);
GYRO_NOISE_DENSITY = 0.01;      % Rate Noise Spectral Density º/s/√Hz

% Digital-output triple-axis accelerometer with a programmable 
% full scale range of ±2g, ±4g, ±8g and ±16g and integrated 16-bit ADCs
% AFS_SEL=0 16,384 LSB/g
% AFS_SEL=1 8,192 LSB/g
% AFS_SEL=2 4,096 LSB/g
% AFS_SEL=3 2,048 LSB/
ACCEL_FSR = 16;
ACCEL_SENS = 2048;
ACCEL_GAIN = (ACCEL_SENS/1000);

% Wing Characteristic








