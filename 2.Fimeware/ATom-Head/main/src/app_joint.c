#include <string.h>
#include <math.h>

#include "app_joint.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_console.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "app_ui.h"
#include "app_sr_handler.h"

static const char *TAG = "joint";

#define ANY 0
#define JOINT_SIZE 1

#define I2C_MASTER_SCL_IO 19        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 47        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 1            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define ACK_CHECK_EN 0x0
#define ESP_SLAVE_ADDR 0x0

#define TIGGER_THRESHOLD 25
// Smoothing filter parameter
#define ALPHA 0.2

// Current angle of the servo
static float currentAngle = 0.0;
// Previous angle of the servo
static float previousAngle = 0.0;
// Smoothed angle of the servo
static float smoothedAngle = 0.0;
// Flag for triggering interface switch
static bool switchFlag = false;
// Direction of interface switch (1 for positive, -1 for negative)
static int switchDirection = 0;

static uint8_t i2cRxData[8];
static uint8_t i2cTxData[8];
struct Joint_device joint[JOINT_SIZE];

static bool delete_task_flag = false;

struct Joint_device *get_joint_obj(void)
{
    return &joint[JOINT_SIZE];
}

int get_switchDirection_status(void)
{
    return switchDirection;
}

void delete_joint_task(void)
{
    delete_task_flag = true;
}

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

void UpdateServoAngle_2(struct Joint_device *_joint, float _angleSetPoint)
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

void UpdateServoAngle_1(struct Joint_device *_joint)
{
    uint8_t *b = (unsigned char *)&(_joint->config.angle);

    i2cTxData[0] = 0x11;

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);
}

void SetJointEnable(struct Joint_device *_joint, bool _enable)
{
    i2cTxData[0] = 0xff;
    i2cTxData[1] = _enable ? 1 : 0;

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);
}

void SetJointTorqueLimit(struct Joint_device *_joint, float _percent)
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

void SetJointId(struct Joint_device *_joint, uint8_t _id)
{
    i2cTxData[0] = 0x21;
    i2cTxData[1] = _id;

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
}

void SetJointInitAngle(struct Joint_device *_joint, float _angle)
{
    float sAngle = _joint->config.inverted ? (_angle - _joint->config.modelAngelMin) /
                                                     (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                                                     (_joint->config.angleMin - _joint->config.angleMax) +
                                                 _joint->config.angleMax
                                           : (_angle - _joint->config.modelAngelMin) /
                                                     (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                                                     (_joint->config.angleMax - _joint->config.angleMin) +
                                                 _joint->config.angleMin;

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

void SetJointKp(struct Joint_device *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x22;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
}

void SetJointKi(struct Joint_device *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x23;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
}

void SetJointKv(struct Joint_device *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x24;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
}

void SetJointKd(struct Joint_device *_joint, float _value)
{
    uint8_t *b = (unsigned char *)(&_value);

    i2cTxData[0] = 0x25;
    for (int i = 0; i < 4; i++)
        i2cTxData[i + 1] = *(b + i);

    TransmitAndReceiveI2cPacket(_joint->config.id);

    _joint->config.angle = *(float *)(i2cRxData + 1);

    vTaskDelay(500 / portTICK_PERIOD_MS); // wait servo reset
}

void updateJointAngle_1(struct Joint_device *_joint)
{
    UpdateServoAngle_1(_joint);

    float jAngle = _joint->config.inverted ? (_joint->config.angleMax - _joint->config.angle) /
                                                     (_joint->config.angleMax - _joint->config.angleMin) *
                                                     (_joint->config.modelAngelMax - _joint->config.modelAngelMin) +
                                                 _joint->config.modelAngelMin
                                           : (_joint->config.angle - _joint->config.angleMin) /
                                                     (_joint->config.angleMax - _joint->config.angleMin) *
                                                     (_joint->config.modelAngelMax - _joint->config.modelAngelMin) +
                                                 _joint->config.modelAngelMin;

    _joint->config.angle = jAngle;
}

void updateJointAngle_2(struct Joint_device *_joint, float _angleSetPoint)
{
    float sAngle = _joint->config.inverted ? (_angleSetPoint - _joint->config.modelAngelMin) /
                                                     (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                                                     (_joint->config.angleMin - _joint->config.angleMax) +
                                                 _joint->config.angleMax
                                           : (_angleSetPoint - _joint->config.modelAngelMin) /
                                                     (_joint->config.modelAngelMax - _joint->config.modelAngelMin) *
                                                     (_joint->config.angleMax - _joint->config.angleMin) +
                                                 _joint->config.angleMin;

    UpdateServoAngle_2(_joint, sAngle);

    float jAngle = _joint->config.inverted ? (_joint->config.angleMax - _joint->config.angle) /
                                                     (_joint->config.angleMax - _joint->config.angleMin) *
                                                     (_joint->config.modelAngelMax - _joint->config.modelAngelMin) +
                                                 _joint->config.modelAngelMin
                                           : (_joint->config.angle - _joint->config.angleMin) /
                                                     (_joint->config.angleMax - _joint->config.angleMin) *
                                                     (_joint->config.modelAngelMax - _joint->config.modelAngelMin) +
                                                 _joint->config.modelAngelMin;

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
    updateJointAngle_2(&joint[ANY], angle);

    ESP_LOGI(TAG, "set dev1 target angle:%f | current angle:%f", angle, joint[ANY].config.angle);

    return ESP_OK;
}

static int jointdev_scan(void)
{
    int err = ESP_OK;
    int joint_id = -1;

    printf("I2C Scanning I2C devices...\n");
    printf("----------------------------\n");

    for (int i = 0; i < 8; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            joint_id = i;
            printf("I2C Device found at address 0x%.2X\n", i);
        }
    }

    if (joint_id < 0)
        err = ESP_FAIL;

    printf("----------------------------\n");
    printf("I2C scanning complete.\n");

    return err;
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

    if ((ret = i2c_param_config(sccb_i2c_port, &conf)) != ESP_OK)
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
            .argtable = NULL};
    ESP_ERROR_CHECK(esp_console_cmd_register(&joint_cmd));
}

void register_jointcmd(void)
{
    register_jointset();
}

#define SERVO_MIN_ANGLE -15
#define SERVO_MAX_ANGLE 40
#define SERVO_PERIOD_MS 220
#define SERVO_ROTATION_TIME_MS 10000

/* 头部关节动作*/
void timer_callback(TimerHandle_t timer)
{
    static int angle = SERVO_MIN_ANGLE;
    const uint8_t ucAngleStep = (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / (SERVO_ROTATION_TIME_MS / SERVO_PERIOD_MS);

    // set joint angle
    updateJointAngle_2(&joint[ANY], angle);

    angle += ucAngleStep;
    if ((int)joint[ANY].config.angle > SERVO_MAX_ANGLE)
    {
        en_sr_detect_task();
        updateJointAngle_2(&joint[ANY], 0);
        xTimerDelete(timer, 0);
    }
}

// Update servo angle and apply smoothing filter
void updateAngle(float newAngle)
{
    previousAngle = currentAngle;
    currentAngle = newAngle;
    smoothedAngle = ALPHA * currentAngle + (1 - ALPHA) * previousAngle;
}

// Check if servo angle exceeds threshold and set the flag and direction
void checkAngleThreshold(void)
{
    float angleDiff = currentAngle - smoothedAngle;

    if (fabs(angleDiff) > TIGGER_THRESHOLD)
    {
        switchFlag = true;
        switchDirection = angleDiff > 0 ? 1 : -1;
    }
    else
    {
        switchFlag = false;
        switchDirection = 0;
    }
}

static void joint_ui_menu_task(void *parm)
{
#define NUM_OF_SCREENS 2

    static uint8_t currentScreen = 1;
    // close control first
    SetJointEnable(&joint[ANY], false);

    while (1)
    {
        if (delete_task_flag == true)
        {
            vTaskDelete(NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(600));

        updateJointAngle_1(&joint[ANY]);

        // Update servo angle and Check if servo angle exceeds threshold
        updateAngle(joint->config.angle);
        checkAngleThreshold();

        // set angle to lcd
        ui_set_joint_angle((int16_t)joint->config.angle);

        if (switchFlag)
        {
            if (switchDirection == 1)
            {
                if (currentScreen < NUM_OF_SCREENS - 1)
                {
                    currentScreen++;
                    ui_set_menu(&currentScreen);
                }
            }
            else if (switchDirection == -1)
            {
                if (currentScreen > 0)
                {
                    currentScreen--;
                    ui_set_menu(&currentScreen);
                }
            }
        }
    }
}

void start_first_action(void)
{
    // create timer
    TimerHandle_t timer = xTimerCreate("servo_timer", pdMS_TO_TICKS(SERVO_PERIOD_MS), pdTRUE, NULL, timer_callback);
    if (timer != NULL)
    {
        xTimerStart(timer, 0);
    }

    // enable control
    joint_init();
    // setup joint angle
    updateJointAngle_2(&joint[ANY], SERVO_MIN_ANGLE);
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

    if (jointdev_scan() != ESP_OK)
    {
        ESP_LOGE(TAG, "not find joint device!");
        return res;
    }

    // init joint
    joint_init();

    // setup joint angle
    updateJointAngle_2(&joint[ANY], SERVO_MIN_ANGLE);

    return res;
}

esp_err_t AppJoint_run(void)
{
    esp_err_t ret = ESP_OK;

    xTaskCreate(joint_ui_menu_task, "joint_task", 4096, NULL, 2, NULL);

    return ret;
}