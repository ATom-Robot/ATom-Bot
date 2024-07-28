# I2Cdev lib for ESP32 (for MPU6050, BMP280)
I2Cdev bus implementation based on JROWBERG implementation. Modified to be more inititalized as an object, improved code consistency, some functional and structural fixes. You can find the original code here: https://github.com/jrowberg/i2cdevlib/

It is prepared to be runned as component with dependency on I2Cdev lib (https://github.com/PiotrTopa/esp32-I2Cdev). It is compatibile with PlatformIO esp-idf framework.

# Add components to your project
You need to add this repository along with I2Cdev to your project as ESP-IDF component.
```bash
cd ~/myProjects/myProject
cd components
git clone https://github.com/PiotrTopa/esp32-I2Cdev I2Cdev
```

# Example
Please see code examples at BMP280 / MPU6050 components. You can find them here:
- https://github.com/PiotrTopa/esp32-BMP280
- https://github.com/PiotrTopa/esp32-MPU6050