#ifndef __STATUS_H__
#define __STATUS_H__

typedef struct {
    char    state[32];
    int	    present, loaded;
    int	    mm;
} lane_status_t;

typedef struct {
    lane_status_t lanes[2];
    int	    uptime;
    int	    active_lane;
    int	    y_output;
    int	    buffer_full, buffer_empty;
} status_t;

static const status_t empty_status = {
    {
	{ "INVALID", 0 },
	{ "INVALID", 0 },
    },
    0,
    -1,
    0,
    0, 0
};

#endif
    
