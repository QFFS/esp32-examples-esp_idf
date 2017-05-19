
#include <stdio.h>
#include "driver/i2c.h"

/**
 * TEST CODE BRIEF
 *
 * This  will show you how to use I2C module by running  a task on i2c bus:
 *
 * - read external i2c sensor, here we use a adxl345 sensor(GY-291 module) for instance.
 * 
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Connection:
 * - connect adxl345 module sda/scl of sensor with GPIO18/GPIO19
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 */

#define I2C_MASTER_SCL_IO    19    /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    18    /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_TX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0   /*!< I2C master do not need buffer */
#define I2C_MASTER_FREQ_HZ    100000     /*!< I2C master clock frequency */

#define EARTH_GRAVITY_MS2    9.80665
#define SCALE_MULTIPLIER     0.004

#define DATA_FORMAT          0x31
#define BW_RATE              0x2C
#define POWER_CTL            0x2D

#define BW_RATE_1600HZ       0x0F
#define BW_RATE_800HZ        0x0E
#define BW_RATE_400HZ        0x0D
#define BW_RATE_200HZ        0x0C
#define BW_RATE_100HZ        0x0B
#define BW_RATE_50HZ         0x0A
#define BW_RATE_25HZ         0x09

#define RANGE_2G             0x00
#define RANGE_4G             0x01
#define RANGE_8G             0x02
#define RANGE_16G            0x03

#define MEASURE              0x08
#define AXES_DATA            0x32
#define ADXL345_SENSOR_ADDR  0x53    /*!< slave address for BH1750 sensor */


#define BH1750_SENSOR_ADDR  0x53    /*!< slave address for BH1750 sensor */
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

/*
*write byte to the slave device register
*/
static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t device_addr, 
                                            uint8_t reg_addr, uint8_t data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, device_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/*
 read register of slave device
*/
static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t device_addr, 
                                            uint8_t reg_addr, uint8_t* data,size_t size)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
     cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, device_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, device_addr << 1 | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/*
*config adxl345
*/
static esp_err_t i2c_adxl345_config()
{
    int ret;
    int errcount = 0;
    
    ret = i2c_master_write_slave_reg(I2C_MASTER_NUM, ADXL345_SENSOR_ADDR,BW_RATE, BW_RATE_100HZ);
    if (ret == ESP_OK)
    {
        errcount++;
    }   
    
    ret = i2c_master_write_slave_reg(I2C_MASTER_NUM, ADXL345_SENSOR_ADDR,DATA_FORMAT, 1<<3);
    if (ret == ESP_OK)
    {
        errcount++;
    } 

    ret = i2c_master_write_slave_reg(I2C_MASTER_NUM, ADXL345_SENSOR_ADDR,POWER_CTL, MEASURE);
    if (ret == ESP_OK)
    {
        errcount++;
    }  

    printf("%d\n",errcount);
    if (errcount == 3)
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }


}
/*
*get x y z acceleration m/s^2
*/
static esp_err_t  getXYZAcc(float *xyz) 
{
    int ret;
    uint8_t buff[6] = {0,0};
    int16_t temp;

    
    ret = i2c_master_read_slave_reg(I2C_MASTER_NUM, ADXL345_SENSOR_ADDR,AXES_DATA, buff, 6);

    if (ret == ESP_OK)
    {
        //x
        temp = (buff[1] << 8) | buff[0];
        xyz[0] = (float)(temp)*0.004*9.8;

        //y
        temp = (buff[3] << 8) | buff[2];
        xyz[1] = (float)(temp)*0.004*9.8;
        
        //z
        temp = (buff[5] << 8) | buff[4];
        xyz[2] = (float)(temp)*0.004*9.8;

        return ret;
    }
    else
        return ESP_FAIL;


}
/**
 * @brief i2c master initialization
 */
static void i2c_master_init()
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void i2c_adxl435_task(void* arg)
{
    float xyz[3];
    while (1)
    {
        if (getXYZAcc(xyz) == ESP_OK)
        {
            printf("---------------get xyz acceleration data success-------------------\n");
            printf("x: %f m/s^2\n", xyz[0]);
            printf("y: %f m/s^2\n", xyz[1]);
            printf("z: %f m/s^2\n", xyz[2]); 
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    int ret;
    i2c_master_init();
    ret = i2c_adxl345_config();
     if (ret == ESP_OK) {
        printf("config write ok!\n");
    }
    xTaskCreate(i2c_adxl435_task, "i2c_test_task_0", 1024 * 2, (void* ) 0, 10, NULL);
}

