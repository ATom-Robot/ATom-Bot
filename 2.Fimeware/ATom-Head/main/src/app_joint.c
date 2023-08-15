#include "app_joint.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_console.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "joint";

#define ANY 0
#define JOINT_SIZE 1

#define I2C_MASTER_SCL_IO           19              /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           47              /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              1               /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0               /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define ACK_CHECK_EN                0x0
#define ESP_SLAVE_ADDR              0x0

static uint8_t i2cRxData[8];
static uint8_t i2cTxData[8];
struct Joint_device joint[JOINT_SIZE];

int I2C_WriteData(uint8_t slaveAddr, uint8_t regAddr, uint8_t *pData, uint16_t dataLen)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slaveAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    if (NULL != regAddr)
    {
        i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    }
    i2c_master_write(cmd, pData, dataLen, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(1, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void TransmitAndReceiveI2cPacket(uint8_t _id)
{
    esp_err_t state;

    state = i2c_master_write_read_device(I2C_MASTER_NUM, _id, i2cTxData, 5, i2cRxData, 5, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

void UpdateServoAngle_2(struct Joint_device  *_joint, float _angleSetPoint)
{
    if (_angleSetPoint >= _joint->config.angleMin && _angleSetPoint <= _joint->config.angleMax)
    {
        uint8_t *b = (unsigned char *)(&_angleSetPoint);

        i2cTxData[0] = 0x01;
        for (int i = 0; i < 4; i++)
            i2cTxData[i + 1] = *(b + i);

        TransmitAndReceiveI2cPacket(_joint->config.id);

        _joint->config.angle = *(float *)(i2cRxData + 1);
    }
}

void UpdateServoAngle_1(struct Joint_device  *_joint)
{
    uint8_t *b = (unsigned char *) & (_joint->config.angle);

    i2cTxData[0] = 0x11;

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);
}

void SetJointEnable(struct Joint_device  *_joint, bool _enable)
{
    i2cTxData[0] = 0xff;
    i2cTxData[1] = _enable ? 1 : 0;

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);
}

void SetJointTorqueLimit(struct Joint_device  *_joint, float _percent)
{
    if (_percent >= 0 && _percent <= 1)
    {
        uint8_t *b = (unsigned char *)(&_percent);

        i2cTxData[0] = 0x26;
        for (int i = 0; i < 4; i++)
            i2cTxData[i + 1] = *(b + i);

        TransmitAndReceiveI2cPacket(_joint->config.id);

        _joint->config.angle = *(float *)(i2cRxData + 1);

        vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
    }
}

void SetJointId(struct Joint_device  *_joint, uint8_t _id)
{
    i2cTxData[0] = 0x21;
    i2cTxData[1] = _id;

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
}

void SetJointInitAngle(struct Joint_device  *_joint, float _angle)
{
    float sAngle = _joint->config.inverted ?
                   (_angle - _joint->config.modelAngelMin) /
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                   (_joint->config.angleMin - _joint->config.angleMax) + _joint->config.angleMax :
                   (_angle - _joint->config.modelAngelMin) /
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                   (_joint->config.angleMax - _joint->config.angleMin) + _joint->config.angleMin;


    if (sAngle >= _joint->config.angleMin && sAngle <= _joint->config.angleMax)
    {
        uint8_t *b = (unsigned char *)(&_angle);

        i2cTxData[0] = 0x27;
        for (int i = 0; i < 4; i++)
            i2cTxData[i + 1] = *(b + i);

        TransmitAndReceiveI2cPacket(_joint->config.id);

        _joint->config.angle = *(float *)(i2cRxData + 1);

        vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
    }
}

void SetJointKp(struct Joint_device  *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x22;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
}

void SetJointKi(struct Joint_device  *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x23;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
}

void SetJointKv(struct Joint_device  *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x24;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
}

void SetJointKd(struct Joint_device  *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x25;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
}

void UpdateJointAngle_1(struct Joint_device  *_joint)
{
    UpdateServoAngle_1(_joint);

    float jAngle = _joint->config.inverted ?
                   (_joint->config.angleMax - _joint->config.angle) /
                   (_joint->config.angleMax - _joint->config.angleMin) *
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) + _joint->config.modelAngelMin :
                   (_joint->config.angle - _joint->config.angleMin) /
                   (_joint->config.angleMax - _joint->config.angleMin) *
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) + _joint->config.modelAngelMin;

    _joint->config.angle = jAngle;
}

void UpdateJointAngle_2(struct Joint_device  *_joint, float _angleSetPoint)
{
    float sAngle = _joint->config.inverted ?
                   (_angleSetPoint - _joint->config.modelAngelMin) /
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                   (_joint->config.angleMin - _joint->config.angleMax) + _joint->config.angleMax :
                   (_angleSetPoint - _joint->config.modelAngelMin) /
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                   (_joint->config.angleMax - _joint->config.angleMin) + _joint->config.angleMin;

    UpdateServoAngle_2(_joint, sAngle);

    float jAngle = _joint->config.inverted ?
                   (_joint->config.angleMax - _joint->config.angle) /
                   (_joint->config.angleMax - _joint->config.angleMin) *
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) + _joint->config.modelAngelMin :
                   (_joint->config.angle - _joint->config.angleMin) /
                   (_joint->config.angleMax - _joint->config.angleMin) *
                   (_joint->config.modelAngelMax - _joint->config.modelAngelMin) + _joint->config.modelAngelMin;

    _joint->config.angle = jAngle;
}

static int set_id_to_joint(int argc, const char *argv[])
{
    if (argc != 2)
    {
        ESP_LOGI(TAG, "error paramter");
        return ESP_FAIL;
    }
    uint8_t id = atoi(argv[1]);
    SetJointId(&joint[ANY], id);

    vTaskDelay(800 / portTICK_PERIOD_MS);
    joint[ANY].config.id = id;
    SetJointEnable(&joint[ANY], true);

    return ESP_OK;
}

static int set_kp_to_joint(int argc, const char *argv[])
{
    if (argc != 2)
    {
        ESP_LOGI(TAG, "error paramter");
        return ESP_FAIL;
    }

    float kp = atof(argv[1]);

    SetJointKp(&joint[ANY], kp);
    vTaskDelay(800 / portTICK_PERIOD_MS);
    SetJointEnable(&joint[ANY], true);

    return ESP_OK;
}

static int set_ki_to_joint(int argc, const char *argv[])
{
    if (argc != 2)
    {
        ESP_LOGI(TAG, "error paramter");
        return ESP_FAIL;
    }

    float kp = atof(argv[1]);

    SetJointKi(&joint[ANY], kp);
    vTaskDelay(800 / portTICK_PERIOD_MS);
    SetJointEnable(&joint[ANY], true);

    return ESP_OK;
}

static int set_kd_to_joint(int argc, const char *argv[])
{
    if (argc != 2)
    {
        ESP_LOGE(TAG, "error paramter");
        return ESP_FAIL;
    }

    float kd = atof(argv[1]);

    SetJointKd(&joint[ANY], kd);
    vTaskDelay(800 / portTICK_PERIOD_MS);
    SetJointEnable(&joint[ANY], true);

    return ESP_OK;
}

static int set_torque_to_joint(int argc, const char *argv[])
{
    if (argc != 2)
    {
        ESP_LOGE(TAG, "error paramter");
        return ESP_FAIL;
    }

    float kd = atof(argv[1]);
    ESP_LOGI(TAG, "%f\n", kd);

    SetJointTorqueLimit(&joint[ANY], kd);
    vTaskDelay(800 / portTICK_PERIOD_MS);
    SetJointEnable(&joint[ANY], true);

    return ESP_OK;
}

static int joint_angele_test(int argc, const char *argv[])
{
    if (argc != 2)
    {
        ESP_LOGE(TAG, "error paramter");
        return ESP_FAIL;
    }

    float angle = atof(argv[1]);

    SetJointEnable(&joint[ANY], true);
    UpdateJointAngle_2(&joint[ANY], angle);

    ESP_LOGI(TAG, "set dev1 target angle:%f | current angle:%f", angle, joint[ANY].config.angle);

    return ESP_OK;
}

static int joint_init(void)
{
    // Left arm
    joint[ANY].config.id = 0;
    joint[ANY].config.angleMin = 0;
    joint[ANY].config.angleMax = 180;
    joint[ANY].config.angle = 0;
    joint[ANY].config.modelAngelMin = -90;
    joint[ANY].config.modelAngelMax = 90;
    joint[ANY].config.inverted = false;

    SetJointEnable(&joint[ANY], true);
    UpdateJointAngle_2(&joint[ANY], 0);

    return ESP_OK;
}

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf;
    esp_err_t ret;

    memset(&conf, 0, sizeof(i2c_config_t));

    static int sccb_i2c_port;
    sccb_i2c_port = I2C_MASTER_NUM;
    ESP_LOGI(TAG, "sccb_i2c_port=%d\n", sccb_i2c_port);

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    if ((ret =  i2c_param_config(sccb_i2c_port, &conf)) != ESP_OK)
    {
        return ret;
    }

    return i2c_driver_install(sccb_i2c_port, conf.mode, 0, 0, 0);
}

static void register_jointset(void)
{
    const esp_console_cmd_t joint_cmd =
    {
        .command = "joint",
        .help = "set joint angle <-180-180>",
        .hint = NULL,
        .func = &joint_angele_test,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&joint_cmd));
}

void register_jointcmd(void)
{
    register_jointset();
}

esp_err_t joint_i2c_init(void)
{
    esp_err_t res = ESP_OK;
    res = i2c_master_init();
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "i2c init fail");
        return res;
    }
    joint_init();
    return res;
}