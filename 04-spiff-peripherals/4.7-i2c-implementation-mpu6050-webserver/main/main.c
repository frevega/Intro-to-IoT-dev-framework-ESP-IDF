#include <stdio.h>
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "cJSON.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include <esp_http_server.h>
#include <stdlib.h>

static const char *TAG = "ESP32-MPU6050";

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA        /*!< gpio number for I2C master data  */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                    /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                    /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */

/*MPU6050 register addresses */

#define MPU6050_REG_POWER 0x6B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define I2C_MASTER_NUM 0

/*These are the addresses of mpu6050 from which you will fetch accelerometer x,y,z high and low values */
#define MPU6050_REG_ACC_X_HIGH 0x3B
#define MPU6050_REG_ACC_X_LOW 0x3C
#define MPU6050_REG_ACC_Y_HIGH 0x3D
#define MPU6050_REG_ACC_Y_LOW 0x3E
#define MPU6050_REG_ACC_Z_HIGH 0x3F
#define MPU6050_REG_ACC_Z_LOW 0x40

/*These are the addresses of mpu6050 from which you will fetch gyro x,y,z high and low values */

#define MPU6050_REG_GYRO_X_HIGH 0x43
#define MPU6050_REG_GYRO_X_LOW 0x44
#define MPU6050_REG_GYRO_Y_HIGH 0x45
#define MPU6050_REG_GYRO_Y_LOW 0x46
#define MPU6050_REG_GYRO_Z_HIGH 0x47
#define MPU6050_REG_GYRO_Z_LOW 0x48

/*MPU6050 address and who am i register*/

#define MPU6050_SLAVE_ADDR 0x68
#define who_am_i 0x75
#define MPU6050_RESET_BIT 7

uint16_t pbuffer[7];
uint8_t buffer[14];

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t mpu_wake(void)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU6050_REG_POWER, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t i2c_master_sensor_test(uint8_t length, uint8_t *data, uint16_t timeout)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MPU6050_REG_ACC_X_HIGH, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    vTaskDelay(30 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SLAVE_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, length - 1, ACK_VAL);
    i2c_master_read(cmd, data, 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void disp_buf(uint8_t *buf, int len)
{
    pbuffer[0] = (int)((buf[0] << 8) | buf[1]);
    pbuffer[1] = (int)((buf[2] << 8) | buf[3]);
    pbuffer[2] = (int)((buf[4] << 8) | buf[5]);
    pbuffer[3] = (int)((buf[6] << 8) | buf[7]);
    pbuffer[4] = (int)((buf[8] << 8) | buf[9]);
    pbuffer[5] = (int)((buf[10] << 8) | buf[11]);
    pbuffer[6] = (int)((buf[12] << 8) | buf[13]);
}

void i2c_task(void *arg)
{
    int ret;
    while (1)
    {
        ret = i2c_master_sensor_test(14, &buffer[0], 0);
        if (ret == ESP_ERR_TIMEOUT)
        {
            printf("\n I2C Timeout");
        }
        else if (ret == ESP_OK)
        {
            printf("*******************\n");
            printf("TASK: MASTER READ SENSOR( MPU6050 )\n");
            printf("*******************\n");
        }
        else
        {
            printf("\n No ack, sensor not connected...skip...");
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
        for (int i = 0; i < 14; i++)
        {
            printf("%d ", buffer[i]);
        }
        printf("\n");
    }
    vTaskDelete(NULL);
}

static esp_err_t mpu6050_rawvalues_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();

    disp_buf(&buffer[0], 14);

    cJSON_AddNumberToObject(root, "accx_raw:", pbuffer[0]);
    cJSON_AddNumberToObject(root, "accy_raw :", pbuffer[1]);
    cJSON_AddNumberToObject(root, "accz_raw :", pbuffer[2]);
    cJSON_AddNumberToObject(root, "temp_raw :", pbuffer[3]);
    cJSON_AddNumberToObject(root, "gyrox_raw :", pbuffer[4]);
    cJSON_AddNumberToObject(root, "gyroy_raw :", pbuffer[5]);
    cJSON_AddNumberToObject(root, "gyroz_raw :", pbuffer[6]);
    const char *sys_info = cJSON_Print(root);
    httpd_resp_sendstr(req, sys_info);
    free((void *)sys_info);
    cJSON_Delete(root);
    return ESP_OK;
}

static const httpd_uri_t hello = {
    .uri = "/api/mpu6050/rawvalues",
    .method = HTTP_GET,
    .handler = mpu6050_rawvalues_get_handler,
    .user_ctx = NULL};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello);
        return server;
    }
    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void stop_webserver(httpd_handle_t server)
{
    httpd_stop(server);
}

static void disconnect_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server)
    {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server == NULL)
    {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(mpu_wake());
    xTaskCreate(i2c_task, "i2c_test_task", 1024 * 2, NULL, 10, NULL);

    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));

    server = start_webserver();
}
