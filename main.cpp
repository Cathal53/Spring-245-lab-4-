/* Includes */
#include "mbed.h"
#include "HTS221Sensor.h"
#include "LPS22HBSensor.h"
#include "LSM6DSLSensor.h"
#include "lis3mdl_class.h"
#include "VL53L0X.h"

// Objects for various sensors
static DevI2C devI2c(PB_11, PB_10);
static LPS22HBSensor press_temp(&devI2c);
static HTS221Sensor hum_temp(&devI2c);
static LSM6DSLSensor acc_gyro(&devI2c, 0xD4, D4, D5); // high address
static LIS3MDL magnetometer(&devI2c, 0x3C);
static DigitalOut shutdown_pin(PC_6);
static VL53L0X range(&devI2c, &shutdown_pin, PC_7, 0x52);
UnbufferedSerial pc(USBTX, USBRX); // USB Serial

// Flags for sensor data printing
volatile bool print_accel_flag = false;
volatile bool print_gyro_flag = false;
volatile bool print_mag_flag = false;
volatile bool print_t_rh_flag = false;
volatile bool print_distance_flag = false;

// Function to handle serial input interrupt
void serial_interrupt() {
    char c;
    if (pc.read(&c, 1)) {
        switch (c) {
            case 'a':
                print_accel_flag = true;
                break;
            case 'g':
                print_gyro_flag = true;
                break;
            case 'm':
                print_mag_flag = true;
                break;
            case 't':
                print_t_rh_flag = true;
                break;
            case 'd':
                print_distance_flag = true;
                break;
        }
    }
}

// Functions to print sensor data
void print_t_rh() {
    float value1, value2;
    hum_temp.get_temperature(&value1);
    hum_temp.get_humidity(&value2);

    value1 = value2 = 0;
    press_temp.get_temperature(&value1);
    press_temp.get_pressure(&value2);
    printf("LPS22HB: [temp] %.2f C, [press] %.2f mbar\r\n", value1, value2);
}

void print_mag() {
    int32_t axes[3];
    magnetometer.get_m_axes(axes);
    printf("LIS3MDL [mag/mgauss]:    %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
}

void print_accel() {
    int32_t axes[3];
    acc_gyro.get_x_axes(axes);
    printf("LSM6DSL [acc/mg]:        %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
}

void print_gyro() {
    int32_t axes[3];
    acc_gyro.get_g_axes(axes);
    printf("LSM6DSL [gyro/mdps]:     %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);
}

void print_distance() {
    uint32_t distance;
    int status = range.get_distance(&distance);
    if (status == VL53L0X_ERROR_NONE) {
        printf("VL53L0X [mm]:            %6ld\r\n", distance);
    } else {
        printf("VL53L0X [mm]:                --\r\n");
    }
}

/* Simple main function */
int main() {
    uint8_t id;
    float value1, value2;

    int32_t axes[3];

    hum_temp.init(NULL);
    press_temp.init(NULL);
    magnetometer.init(NULL);
    acc_gyro.init(NULL);
    range.init_sensor(0x52);

    hum_temp.enable();
    press_temp.enable();
    acc_gyro.enable_x();
    acc_gyro.enable_g();

    pc.baud(9600);
    pc.attach(&serial_interrupt);

    printf("\033[2J\033[20A");
    printf("\r\n--- Starting new run ---\r\n\r\n");

    hum_temp.read_id(&id);
    printf("HTS221  humidity & temperature    = 0x%X\r\n", id);

    press_temp.read_id(&id);
    printf("LPS22HB pressure & temperature    = 0x%X\r\n", id);
    magnetometer.read_id(&id);
    printf("LIS3MDL magnetometer              = 0x%X\r\n", id);
    acc_gyro.read_id(&id);
    printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);

    printf("\n\r--- Reading sensor values ---\n\r");

    while (1) {
        if (print_accel_flag) {
            print_accel();
            print_accel_flag = false;
        }
        if (print_gyro_flag) {
            print_gyro();
            print_gyro_flag = false;
        }
        if (print_mag_flag) {
            print_mag();
            print_mag_flag = false;
        }
        if (print_t_rh_flag) {
            print_t_rh();
            print_t_rh_flag = false;
        }
        if (print_distance_flag) {
            print_distance();
            print_distance_flag = false;
        }
        wait_us(500000); // 500 ms delay
    }
}