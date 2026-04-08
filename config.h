#ifndef __CONFIG_H__
#define __CONFIG_H__

typedef struct {
    int	    present;
    int	    loaded;
    int	    enable;
    int	    dir;
    int	    step;
    int	    uart_address;
    int     invert;
} lane_config_t;

typedef struct {
    int	    tx, rx;
    int	    rms_current;
    int	    microstepping;
    int	    steps_per_mm;
    int	    preload_speed;
    int	    loading_speed;
    int	    refill_speed;
} motor_config_t;

typedef struct {
    int	    input;
    int	    full, empty;
} buffer_config_t;

typedef struct {
    int		   rgb;
    int            mm_to_load, mm_to_retry, mm_to_load2;
    int            y_output_timeout_us;
    int            y_output_retract_mm;
} error_config_t;

typedef struct {
    lane_config_t   lanes[2];
    motor_config_t  motor_config;
    buffer_config_t buffer;
    error_config_t  error;
} config_t;

typedef enum {
    SKR_THB = 26,
    SKR_TH0 = 27,
    SKR_X_STOP = 4,
    SKR_Y_STOP = 3,
    SKR_Z_STOP = 25,
    SKR_RGB = 24,
    SKR_E0_STOP = 16,
    SKR_SERVOS = 29,
    SKR_PROBE = 22,
    SKR_HB = 21,
    SKR_HE = 23,
    SKR_LASER = 20,
    SKR_FAN1 = 17,
    SKR_FAN2 = 18,
    SKR_FAN3 = 20,
} skr_pin_t;

static const config_t factory_config = {
    {
        {
            .present = SKR_PROBE,
            .loaded = SKR_Y_STOP,
            .enable = 15,
            .dir = 13,
            .step = 14,
            .uart_address = 3,
            .invert = false },
        {
            .present = SKR_SERVOS,
            .loaded =  SKR_Z_STOP,
            .enable = 12,
            .dir = 10,
            .step = 11,
            .uart_address = 0,
            .invert = false
        },
    },
    {
        .tx = 8, .rx = 9,
        .rms_current = 850,
        .microstepping = 4,
        .steps_per_mm = 680,
        .preload_speed = 5,
        .loading_speed = 20,
        .refill_speed = 10
    },
    {
        .input = SKR_THB,
        .full = SKR_TH0,
        .empty = SKR_X_STOP,
    },
    {
	.rgb = SKR_RGB,
        .mm_to_load = 200,
        .mm_to_retry = 50,
        .mm_to_load2 = 100,
        .y_output_timeout_us = 10*1000*1000,
        .y_output_retract_mm = 10,
    },
};

static const uint32_t CONFIG_VERSION = 2;

#endif
