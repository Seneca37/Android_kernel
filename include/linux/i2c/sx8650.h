
#ifndef __LINUX_I2C_SX8650_H
#define __LINUX_I2C_SX8650_H

/* linux/i2c/sx8650.h */

struct sx8650_platform_data {
        u16     model;                          /* 8650 */
        u16     y_plate_ohms;

        int     (*get_pendown_state)(void);
        void    (*clear_penirq)(void);          /* If needed, clear 2nd level
                                                   interrupt source */
        int     (*init_platform_hw)(void);
        void    (*exit_platform_hw)(void);
};

#endif



