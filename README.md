# STM32_HAL_PCA9685_driver


Connecting: GND (GND)\
            SCL (SCL on STM32)\
            SDA (SDA on STM32)\
            0E (open)\
            VCC (3.3V)\
            V+ (open)
            
            
Device address is 0x80 (in arduino it is 0x40, maybe Arduino automatically right shift 1 bit for 7 bits I2C address).


Three functions should be called\
begin();                // initialization\
setPWMFreq(50);         // control frequency of PWM (50Hz for servo control)\
set_pwm(&hi2c1, PCA9685_I2C_ADDRESS, 0, 0, value);	//set PCA9685 channel: 1, i2c device handle. 2. I2c device address. 3. channel. 4. start of cycle. 5. end of cycle
