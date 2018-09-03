#pragma once

using namespace math;

class MPU9250 : public InertialSensor
{
public:
    MPU9250();

    bool init(void);
    void read();
    

private:
    
    #define SELF_TEST_X_GYRO        0X00
    #define SELF_TEST_Y_GYRO        0X01
    #define SELF_TEST_Z_GYRO        0X02

    #define SELF_TEST_X_ACCEL       0X0D
    #define SELF_TEST_Y_ACCEL       0X0E
    #define SELF_TEST_Z_ACCEL       0X0F

    #define XG_OFFSET_H             0X13
    #define XG_OFFSET_L             0X14
    #define YG_OFFSET_H             0X15
    #define YG_OFFSET_L             0X16
    #define ZG_OFFSET_H             0X17
    #define ZG_OFFSET_L             0X18

    #define SMPLRT_DIV              0X19 
    #define CONFIG                  0X1A 
    #define GYRO_CONFIG             0X1B 
    #define ACCEL_CONFIG            0X1C 
    #define ACCEL_CONFIG2           0X1D 

    #define LP_ACCEL_ODR            0X1E
    #define WOM_THR                 0X1F
    #define FIFO_EN                 0X23

    #define ACCEL_XOUT_H            0X3B  
    #define ACCEL_XOUT_L            0X3C
    #define ACCEL_YOUT_H            0X3D
    #define ACCEL_YOUT_L            0X3E
    #define ACCEL_ZOUT_H            0X3F
    #define ACCEL_ZOUT_L            0X40

    #define TEMP_OUT_H              0X41  
    #define TEMP_OUT_L              0X42

    #define GYRO_XOUT_H             0X43  
    #define GYRO_XOUT_L             0X44
    #define GYRO_YOUT_H             0X45
    #define GYRO_YOUT_L             0X46
    #define GYRO_ZOUT_H             0X47
    #define GYRO_ZOUT_L             0X48

    #define MAG_XOUT_L              0X03
    #define MAG_XOUT_H              0X04
    #define MAG_YOUT_L              0X05
    #define MAG_YOUT_H              0X06
    #define MAG_ZOUT_L              0X07
    #define MAG_ZOUT_H              0X08

    #define PWR_MGMT_1              0X6B 
    #define PWR_MGMT_2              0X6C 

    #define WHO_AM_I                0X75 
    #define WHO_AM_MAG              0X00 
    #define USER_CTRL               0X6A 

    /*******************************************************************/
    #define MAG_I2C_MST_CTRL        0x24
    #define MAG_I2C_MST_DELAY_CTRL  0x67

    #define MAG_I2C_SLVx_EN         0x80
    #define MAG_I2C_SLV0_ADDR       0x25
    #define MAG_I2C_SLV0_REG        0x26
    #define MAG_I2C_SLV0_CTRL       0x27
    #define MAG_I2C_SLV0_DO         0x63
    #define MAG_I2C_SLV0_NACK       0x01

    #define MAG_I2C_SLV4_DONE       0x40
    #define MAG_I2C_SLV4_ADDR       ((uint8_t)0x31)
    #define MAG_I2C_SLV4_REG        ((uint8_t)0x32)
    #define MAG_I2C_SLV4_DO         ((uint8_t)0x33)
    #define MAG_I2C_SLV4_CTRL       ((uint8_t)0x34)
    #define MAG_I2C_SLV4_DI         ((uint8_t)0x35)


    #define MAG_I2C_MST_STATUS      0x36
    #define MAG_INT_PIN_CFG         0x37
    #define MAG_INT_ENABLE          0x38
    #define MAG_I2C_ADDR            0x0C

    /* Status */
    #define AK8963_STATUS_DRDY      ((uint8_t)0x01)
    #define AK8963_STATUS_DOR       ((uint8_t)0x02)
    #define AK8963_STATUS_HOFL      ((uint8_t)0x08)

    // Read-only Reg
    #define AK8963_WIA              ((uint8_t)0x00)
    #define AK8963_INFO             ((uint8_t)0x01)
    #define AK8963_ST1              ((uint8_t)0x02)
    #define AK8963_ST2              ((uint8_t)0x09)

    #define AK8963_HXL              ((uint8_t)0x03)
    #define AK8963_HXH              ((uint8_t)0x04)
    #define AK8963_HYL              ((uint8_t)0x05)
    #define AK8963_HYH              ((uint8_t)0x06)
    #define AK8963_HZL              ((uint8_t)0x07)
    #define AK8963_HZH              ((uint8_t)0x08)

    // Write/Read Reg
    #define AK8963_CNTL1            ((uint8_t)0x0A)
    #define AK8963_CNTL2            ((uint8_t)0x0B)
    #define AK8963_ASTC             ((uint8_t)0x0C)
    #define AK8963_TS1              ((uint8_t)0x0D)
    #define AK8963_TS2              ((uint8_t)0x0E)
    #define AK8963_I2CDIS           ((uint8_t)0x0F)

    // Read-only Reg ( ROM )
    #define AK8963_ASAX             ((uint8_t)0x10)
    #define AK8963_ASAY             ((uint8_t)0x11)
    #define AK8963_ASAZ             ((uint8_t)0x12)

    #define MAG_EXT_SENS_DATA_00    0x49  
    #define MAG_EXT_SENS_DATA_01    0x4a  
    #define MAG_EXT_SENS_DATA_02    0x4b  
    #define MAG_EXT_SENS_DATA_03    0x4c  

    #define SPI_NCC_PIN     GPIO_Pin_2
    #define SPI_NCC_PORT    GPIOC

    #define MPU_9250_DISENABLE  SPI_NCC_PORT->ODR |=  SPI_NCC_PIN
    #define MPU_9250_ENABLE  SPI_NCC_PORT->ODR &=~ SPI_NCC_PIN    

 

    enum gyro_fsr_e {
        INV_FSR_250DPS = 0,
        INV_FSR_500DPS,
        INV_FSR_1000DPS,
        INV_FSR_2000DPS,
        NUM_GYRO_FSR
    };

    enum accel_fsr_e {
        INV_FSR_2G = 0,
        INV_FSR_4G,
        INV_FSR_8G,
        INV_FSR_16G,
        NUM_ACCEL_FSR
    };    

    
    //I2C* _i2c; 
    
    
    #define MPU9250_ACCEL_DEFAULT_RATE	1000
    #define MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 30
    #define MPU9250_GYRO_DEFAULT_RATE	1000
    #define MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ 30    
    
    
    void SPI1_SetSpeed(uint8_t SPI_BaudRatePrescaler);
    uint8_t MPU9250_Read_Reg(uint8_t reg);
    uint8_t MPU9250_Write_Reg(uint8_t reg,uint8_t value);
    uint8_t MPU9250_Read_Regs(uint8_t reg, uint8_t *buf,uint8_t num);
    uint8_t SPI1_ReadWriteByte(uint8_t TxData,uint8_t *byte);
};

