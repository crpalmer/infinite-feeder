#include "pi.h"
#include <cstring>
#include <math.h>
#include "gp-input.h"
#include "gp-output.h"
#include "pi-threads.h"
#include "stepper.h"
#include "string-utils.h"
#include "thread-interrupt-notifier.h"
#include "time-utils.h"
#include "tmc2209.h"
#include "uart.h"

typedef struct {
    int	    present;
    int	    loaded;
    int	    enable;
    int	    dir;
    int	    step;
    int	    uart_address;
    bool    invert;
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
    lane_config_t   lanes[2];
    motor_config_t  motor_config;
    buffer_config_t buffer;
} config_t;

static config_t config = {
    {
	{
	    .present = 26,
	    .loaded = 27,
	    .enable = 15,
	    .dir = 13,
	    .step = 14,
	    .uart_address = 3,
	    .invert = false },
	{
	    .present = 4,
	    .loaded = 3,
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
	.input = 25,
	.full = 16,
	.empty = 22
    },
};

#define ACTIVE_INIT_MM 2500

// -------------------------- END CONFIG --------------------------

#define TRACE_STATE(x) printf("%s => %s\n", name, x)

class Switch {
public:
    Switch(int pin, ThreadInterruptNotifier *notifier) : pin(pin) {
	input = new GPInput(pin);
	input->set_pullup_up();
	input->set_debounce_us(50);
	input->set_is_inverted();
	input->set_notifier(notifier);
	update();
    }

    bool update() {
	bool new_value = input->get();
	bool updated = new_value != value;
	value = new_value;
	return updated;
    }

    bool get() {
	return value;
    }

private:
    int pin;
    Input *input;
    bool value;
};

class LaneSwitches {
public:
    LaneSwitches(lane_config_t *config, ThreadInterruptNotifier *notifier) {
	present_switch = new Switch(config->present, notifier);
	loaded_switch = new Switch(config->loaded, notifier);
	update();
    }

    bool update() {
	bool p = present_switch->update();
	bool l = loaded_switch->update();
	return p || l;
    }

    bool is_present() { return present_switch->get(); }
    bool is_loaded() { return loaded_switch->get(); }

    void dump_state() {
	if (is_present()) printf(" is-present");
	if (is_loaded()) printf(" is-loaded");
    }

private:
    Switch *present_switch;
    Switch *loaded_switch;
};

class BufferSwitches {
public:
    BufferSwitches(buffer_config_t *config, ThreadInterruptNotifier *notifier) {
	y_output_switch = new Switch(config->input, notifier);
	buffer_full_switch = new Switch(config->full, notifier);
	buffer_empty_switch = new Switch(config->empty, notifier);
	update();
    }

    bool update() {
	bool o = y_output_switch->update();
	bool f = buffer_full_switch->update();
	bool e = buffer_empty_switch->update();
	return o || f || e;
    }

    bool has_y_output() { return y_output_switch->get(); }
    bool buffer_is_full() { return buffer_full_switch->get(); }
    bool buffer_is_empty() { return buffer_empty_switch->get(); }

    void dump_state() {
	if (has_y_output()) printf(" has-y-output");
	if (buffer_is_full()) printf(" buffer-full");
	if (buffer_is_empty()) printf(" buffer-empty");
    }

private:
    Switch *y_output_switch;
    Switch *buffer_full_switch;
    Switch *buffer_empty_switch;
};

class LaneStateNotifier {
public:
    virtual void lane_is_inactive(class Lane *) = 0;
    virtual void lane_is_ready(class Lane *) = 0;
};

class Lane {
public:
    Lane(LaneSwitches *lane_switches, BufferSwitches *buffer_switches, Stepper *stepper, const char *name) : name(name), lane_switches(lane_switches), buffer_switches(buffer_switches), stepper(stepper) {
    }

    void update() {
	bool is_present = lane_switches->is_present();
        bool is_loaded = lane_switches->is_loaded();
	bool buffer_is_full = buffer_switches->buffer_is_full();
	bool buffer_is_empty = buffer_switches->buffer_is_empty();
	bool has_y_output = buffer_switches->has_y_output();

	int feed;
	enum State old_state;

	do {
	    feed = 0;
	    old_state = state;

	    switch (state) {
	    case INIT:
		if (lane_switches->is_loaded()) state = EARLY_ACTIVE_INIT;
		else state = EMPTY;
		printf("%s: initial state: %s\n", name, state_to_string(state));
		break;

	    case EMPTY:
		if (is_present) state = PRE_LOADING;
		break;
	    case PRE_LOADING:
		// TODO: add a timeout
		if (! is_present) state = EMPTY;
		else if (is_loaded) state = PRE_LOADING_RETRACT;
		else feed = config.motor_config.preload_speed;
		break;
	    case PRE_LOADING_RETRACT:
		if (! is_present) state = EMPTY;
		else if (! is_loaded) state = READY;
		else feed = -config.motor_config.preload_speed;
		break;
	    case READY:
		break;
	    case ACTIVATING:
		if (is_loaded) state = LOADING;
		else if (! is_present) state = EMPTY;
		else feed = config.motor_config.loading_speed;
		stepper->reset_n_steps();
		break;
	    case LOADING:
		// TODO: add a timeout in case the filament just isn't loadable and then do something (what??)
		if (! is_loaded && ! has_y_output) state = EMPTY;
		else if (has_y_output) state = EARLY_ACTIVE_INIT;
		// TODO: else if (buffer_is_empty) really short filament in here somewhere??
		else feed = config.motor_config.loading_speed;
		break;
	    case EARLY_ACTIVE_INIT:
		active_init_until = stepper->get_n_steps() + (ACTIVE_INIT_MM * config.motor_config.steps_per_mm);
		state = EARLY_ACTIVE;
		break;
	    case EARLY_ACTIVE:
		if (! is_loaded) state = EMPTYING;
		else if (stepper->get_n_steps() >= active_init_until) state = ACTIVE;
		else if (buffer_is_full) state = EARLY_ACTIVE_WAITING;
		else feed = config.motor_config.refill_speed;
		break;
	    case EARLY_ACTIVE_WAITING:
		if (! is_loaded) state = EMPTYING;
		else if (! buffer_is_full) state = EARLY_ACTIVE;
		break;
	    case ACTIVE:
		if (! is_loaded) state = EMPTYING;
		else if (buffer_is_full) state = WAITING;
		else feed = config.motor_config.refill_speed;
		break;
	    case WAITING:
		if (! is_loaded) state = EMPTYING;
		else if (buffer_is_empty) state = ACTIVE;
		break;
	    case EMPTYING:
		if (! has_y_output) state = EMPTY;
		break;
	    case STOP:
		break;
	    case FEED:
		if (! is_loaded) state = STOP;
		else if (buffer_is_full) state = FEED_WAITING;
		else feed = manual_feed;
		break;
	    case FEED_WAITING:
		if (! is_loaded) state = STOP;
		else if (! buffer_is_full) state = FEED;
		break;
	    case RETRACT:
		if (! is_present) state = STOP;
		else if (buffer_is_empty) state = RETRACT_WAITING;
		else feed = manual_feed;
		break;
	    case RETRACT_WAITING:
		if (! is_present) state = STOP;
		else if (! buffer_is_empty) state = RETRACT;
		break;
	    }

	    trace_state(old_state);
        } while (state != old_state);

	stepper->set_speed(feed);
    }

public:
    bool is_active() {
	return state > READY;
    }

    bool is_ready() {
	return state == READY;
    }

    void activate() {
	assert (state == READY);
	state = ACTIVATING;
	update();
    }

    void dump_state() {
	printf("%s: %s", name, state_to_string(state));
	lane_switches->dump_state();
	if (is_active()) buffer_switches->dump_state();
	printf(" << ");
	stepper->dump_state();
	printf(" >>");
    }

    void error() {
	stepper->set_speed(0);
	state = STOP;
    }

    void stop() {
	state = STOP;
    }

    void feed(int mm_per_sec = 10) {
	manual_feed = mm_per_sec;
	state = FEED;
    }

    void retract(int mm_per_sec = 10) {
	manual_feed = -mm_per_sec;
	state = RETRACT;
    }

    void resume() {
	if (state >= STOP) state = INIT;
    }

    void jump_to_active() {
	if (state >= LOADING && state < ACTIVE) state = ACTIVE;
    }

private:
    const char *name;
    LaneSwitches *lane_switches;
    BufferSwitches *buffer_switches;
    Stepper *stepper;

    int64_t active_init_until = 0;
    int manual_feed = 0;

    enum State {
	    INIT,
	    EMPTY, PRE_LOADING, PRE_LOADING_RETRACT, READY,
	    ACTIVATING, LOADING,
	    EARLY_ACTIVE_INIT, EARLY_ACTIVE, EARLY_ACTIVE_WAITING,
	    ACTIVE, WAITING,
	    EMPTYING,
	    STOP, FEED, FEED_WAITING, RETRACT, RETRACT_WAITING
	} state = INIT;

private:
    const char *state_to_string(enum State state) {
	switch (state) {
	case INIT: return "init";
	case EMPTY: return "empty";
	case PRE_LOADING: return "pre-loading";
	case PRE_LOADING_RETRACT: return "pre-loading(retract)";
	case READY: return "ready";
	case ACTIVATING: return "activating";
	case LOADING: return "loading";
	case EARLY_ACTIVE_INIT: return "init(early)";
	case EARLY_ACTIVE: return "active(early)";
	case EARLY_ACTIVE_WAITING: return "waiting(early)";
	case ACTIVE: return "active";
	case WAITING: return "waiting";
	case EMPTYING: return "emptying";
	case STOP: return "stop";
	case FEED: return "feed";
	case FEED_WAITING: return "feed-waiting";
	case RETRACT: return "retract";
	case RETRACT_WAITING: return "retract-waiting";
	}
	return "** INVALID STATE**";
    }

    inline void trace_state(enum State old_state) {
	if (state != old_state) printf("%s: %s => %s\n", name, state_to_string(old_state), state_to_string(state));
    }
};

// ---------------------------- MAIN -----------------------------

static void configure_tmc(UART_Tx *tx, int address) {
    TMC2209 *tmc = new TMC2209(tx, address);
    tmc->set_microstepping(config.motor_config.microstepping * config.motor_config.steps_per_mm / 16);
    tmc->set_rms_current(config.motor_config.rms_current);
}

static Stepper *create_lane_stepper(lane_config_t *config, motor_config_t *motor_config, const char *name, UART_Tx *tx) {
    Output *enable = new GPOutput(config->enable);
    Output *dir = new GPOutput(config->dir);
    Output *step = new GPOutput(config->step);
    dir->set_is_inverted(config->invert);
    configure_tmc(tx, config->uart_address);
    Stepper *stepper = new Stepper(enable, dir, step, name);
    stepper->set_steps_per_mm(motor_config->steps_per_mm);
    return stepper;
}

class Coordinator : ThreadInterruptNotifier {
public:
    Coordinator() : ThreadInterruptNotifier("coordinator") {
	tx = pico_new_pio_uart_tx(config.motor_config.tx, 115200);
	buffer_switches = new BufferSwitches(&config.buffer, this);
	lane_1_switches = new LaneSwitches(&config.lanes[0], this);
	lane_2_switches = new LaneSwitches(&config.lanes[1], this);
	lane_1 = new Lane(lane_1_switches, buffer_switches, create_lane_stepper(&config.lanes[0], &config.motor_config, "stepper-1", tx), "lane-1");
	lane_2 = new Lane(lane_2_switches, buffer_switches, create_lane_stepper(&config.lanes[1], &config.motor_config, "stepper-2", tx), "lane-2");

	update(true);

	// TODO: What to do here!?!?!
	while (lane_1->is_active() && lane_2->is_active()) {
	    lane_1->error();
	    lane_2->error();
	    printf("\n\nFATAL ERROR: both lanes think they are active!!\n");
	    ms_sleep(1000);
	}

	if (lane_1->is_active()) active_lane = lane_1;
	if (lane_2->is_active()) active_lane = lane_2;
    }

    void on_change_safe() override {
	update();
    }

    void update(bool force = true) {
	bool output_changed = buffer_switches->update();
	bool l1_changed = lane_1_switches->update();
	bool l2_changed = lane_2_switches->update();

	if (force || l1_changed || (active_lane == lane_1 && output_changed)) lane_1->update();
	if (force || l2_changed || (active_lane == lane_2 && output_changed)) lane_2->update();

	if (active_lane && ! active_lane->is_active()) active_lane = NULL;
	activate();
    }

    void activate(bool jump_to_active = false) {
	if (! active_lane) {
	    if (lane_1->is_ready()) active_lane = lane_1;
	    else if (lane_2->is_ready()) active_lane = lane_2;
	    if (active_lane) {
		active_lane->activate();
		printf("activated: ");
		active_lane->dump_state();
	    }
	}
	if (active_lane && jump_to_active) {
	    active_lane->jump_to_active();
	    update(true);
	}
    }
	
    void dump_state() {
	printf("======== Current State ===============\n");
	if (active_lane) printf("%s: ", lane_1 == active_lane ? "ACTIVE" : "      ");
	lane_1->dump_state();
	printf("\n");
	if (active_lane) printf("%s: ", lane_2 == active_lane ? "ACTIVE" : "      ");
	lane_2->dump_state();
	printf("\n");
	printf("all switches: lane_1:");
	lane_1_switches->dump_state();
	printf(" || lane_2:");
	lane_2_switches->dump_state();
	printf(" || output:");
	buffer_switches->dump_state();
	printf("\n");
    }

    void stop() {
	lane_1->stop();
	lane_2->stop();
	update(true);
    }

    void feed(int lane, int speed) {
	if (lane == 1) lane_1->feed(speed);
	else if (lane == 2) lane_2->feed(speed);
	else if (lane < 0 && active_lane) active_lane->feed(speed);
	update(true);
    }

    void retract(int lane, int speed) {
	if (lane == 0) lane_1->retract(speed);
	else if (lane == 2) lane_2->retract(speed);
	else if (lane < 0 && active_lane) active_lane->retract(speed);
	update(true);
    }

    void resume() {
	lane_1->resume();
	lane_2->resume();
	update(true);
    }

private:
    UART_Tx *tx;
    BufferSwitches *buffer_switches;
    LaneSwitches *lane_1_switches;
    LaneSwitches *lane_2_switches;
    Lane *lane_1;
    Lane *lane_2;

    Lane *active_lane = NULL;
};

class StateDumper : public PiThread {
public:
    StateDumper(Coordinator *coordinator) : PiThread("state-dumper"), coordinator(coordinator) {
	lock = new PiMutex();
	cond = new PiCond();
	start();
    }

    void main(void) {
	lock->lock();
	while (1) {
	    while (! enabled) {
		cond->wait(lock);
	    }
	    coordinator->dump_state();
	    ms_sleep(5000);
	}
    }

    void enable() {
	enabled = true;
	cond->signal();
    }

    void disable() {
	enabled = false;
    }

private:
    Coordinator *coordinator;
    PiMutex *lock;
    PiCond *cond;
    bool enabled = false;
};

static void threads_main(int argc, char **argv) {
    ms_sleep(2000);
    printf("Starting\n");
    Coordinator *coordinator = new Coordinator();
    StateDumper *state_dumper = new StateDumper(coordinator);

    printf("Created coordinator, entering interactive loop.\n");
    while (1) {
	static char line[1024];

	if (pi_readline(line, sizeof(line)) != NULL) {
	    if (strcmp(line, "bootsel") == 0) {
		printf("Rebooting to bootloader.\n"); fflush(stdout);
		pi_reboot_bootloader();
	    } else if (strcmp(line, "state") == 0) {
		coordinator->dump_state();
	    } else if (strcmp(line, "threads") == 0 || strcmp(line, "t") == 0) {
		pi_threads_dump_state();
	    } else if (strncmp(line, "state-dumper", 12) == 0 || strcmp(line, "s") == 0) {
		int enabled = true;
		sscanf(&line[12], "%d", &enabled);
		if (enabled) state_dumper->enable();
		else state_dumper->disable();
	    } else if (strcmp(line, "active") == 0) {
		coordinator->resume();
		coordinator->activate(true);
	    } else if (strcmp(line, "stop") == 0) {
		coordinator->stop();
	    } else if (strcmp(line, "resume") == 0) {
		coordinator->resume();
	    } else if (strncmp(line, "feed", 5) == 0) {
		int lane = -1;
		int speed = config.motor_config.loading_speed;
		sscanf(&line[4], "%d %d", &lane, &speed);
		coordinator->feed(lane, speed);
	    } else if (strncmp(line, "retract", 7) == 0) {
		int lane = -1;
		int speed = config.motor_config.loading_speed;
		sscanf(&line[7], "%d %d", &lane, &speed);
		coordinator->retract(lane, speed);
	    } else if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
		printf("bootsel: reboot to bootloader mode\n");
		printf("state: dump state\n");
		printf("threads: dump thread state\n");
		printf("\n");
		printf("active: exit stop / early feeding to move to active\n");
		printf("stop: switch to manual processing\n");
		printf("feed [lane [speed]]: cause a lane to start feeding filament\n");
		printf("retract [lane [speed]]: cause a lane to start retracting filament\n");
		printf("resume: go back to normal processing\n");
	    } else {
		printf("help or ? for usage\n");
	    }
	}
    }
}

int main(int argc, char **argv) {
    pi_init_with_threads(threads_main, argc, argv);
    return 0;
}
