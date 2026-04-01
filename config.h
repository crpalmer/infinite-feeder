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

static const config_t factory_config = {
    {
        {
            .present = 22,
            .loaded = 27,
            .enable = 15,
            .dir = 13,
            .step = 14,
            .uart_address = 3,
            .invert = false },
        {
            .present = 3,
            .loaded =  25,
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
        .input = 16,
        .full = 26,
        .empty = 4,
    },
    {
	.rgb = 24,
        .mm_to_load = 200,
        .mm_to_retry = 50,
        .mm_to_load2 = 100,
        .y_output_timeout_us = 10*1000*1000,
        .y_output_retract_mm = 10,
    },
};

static const uint32_t CONFIG_VERSION = 2;

#endif
