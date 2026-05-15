#include "pi.h"
#include <cstring>
#include <math.h>
#include <limits.h>
#include "config.h"
#include "fram-mb85c.h"
#include "gp-input.h"
#include "gp-output.h"
#include "i2c.h"
#include "neopixel-pico.h"
#include "pi-threads.h"
#include "stepper.h"
#include "string-utils.h"
#include "thread-interrupt-notifier.h"
#include "time-utils.h"
#include "tmc2209.h"

static uint32_t CONFIG_MAGIC = 0x98765432;
static int CONFIG_OFFSET = 0;
static int CONFIG_VERSION_OFFSET = CONFIG_OFFSET + sizeof(CONFIG_MAGIC);
static int CONFIG_DATA_OFFSET = CONFIG_VERSION_OFFSET + sizeof(CONFIG_VERSION);

static us_time_t RETRACT_ON_AFTER_US = 5*1000*1000;

static config_t config = factory_config;
static class Coordinator *coordinator;
static us_time_t boot_at;

class Lights {
public:
    Lights() {
	neo = new NeoPixelPico(config.error.rgb);
	lock = new PiMutex();
	neo->set_n_leds(2);
	neo->set_all(0, 0, 0);
	neo->show();
    }

    void set(int bulb, int r, int g, int b) {
	lock->lock();
	neo->set_led(bulb, r, g, b);
	neo->show();
	lock->unlock();
    }

private:
    NeoPixelPico *neo;
    PiMutex *lock;
};

class LaneLight {
public:
    LaneLight(Lights *lights, int bulb) : lights(lights), bulb(bulb) {
    }

    void empty() { lights->set(bulb, 0, 0, 0); }
    void preloading() { lights->set(bulb, 255, 255, 0); }
    void ready() { lights->set(bulb, 0, 0, 255); }
    void error() { lights->set(bulb, 255, 0, 0); }
    void loading() { lights->set(bulb, 160, 255, 160); }
    void emptying() { lights->set(bulb, 255, 0, 200); }
    void active() { lights->set(bulb, 0, 255, 0); }
    void waiting() { lights->set(bulb, 0, 128, 0); }
    void retract() { lights->set(bulb, 255, 180, 0); }
    void stop() { error(); }

private:
    Lights *lights;
    int bulb;
};

class PersistentStorage {
public:
    PersistentStorage() {
	i2c_init_bus(I2C_BUS, I2C_SDA, I2C_SCL);
	fram = new FRAM_MB85C(I2C_BUS);
    }

    bool load(config_t *c) {
	uint32_t magic;
	if (! fram->read(CONFIG_OFFSET, &magic, sizeof(magic))) return false;
	if (magic != CONFIG_MAGIC) return false;

	uint32_t version;
	if (! fram->read(CONFIG_VERSION_OFFSET, &version, sizeof(version))) return false;

	config_t new_c;
	if (! fram->read(CONFIG_DATA_OFFSET, &new_c, sizeof(new_c))) return false;
	*c = new_c;

	if (version == 2) {
	    printf("Upgrading config version 2 to 3\n");
	    c->retract = factory_config.retract;
	    version++;
	}

	if (version != CONFIG_VERSION) {
	    printf("Loaded version %u of the configuration (should be %u)\n", (unsigned) version, (unsigned) CONFIG_VERSION);
	    return false;
	}

	return true;
    }

    bool save(const config_t *c) {
	uint32_t magic = CONFIG_MAGIC;
	if (! fram->write(CONFIG_OFFSET, &magic, sizeof(magic))) return false;
	uint32_t version = CONFIG_VERSION;
	if (! fram->write(CONFIG_VERSION_OFFSET, &version, sizeof(version))) return false;
	if (! fram->write(CONFIG_DATA_OFFSET, c, sizeof(*c))) return false;
	pi_reboot();
	return true;
    }

    static bool exists() {
#if 0
	i2c_init_bus(I2C_BUS, I2C_SDA, I2C_SCL);
	return (i2c_exists(I2C_BUS, 0x50));
#else
	return false;
#endif
    }

private:
    FRAM *fram;

    static const int I2C_BUS = 0;
    static const int I2C_SDA = 0;
    static const int I2C_SCL = 1;
    static const int CONFIG_OFFSET = 0;
};

static PersistentStorage *storage = NULL;

static void read_int(bool validate_only, const char *prompt, int *value, int min = -INT_MAX, int max = INT_MAX) {
    char line[128];
    int new_value;

    if (validate_only || ! storage) {
	new_value = *value;
    } else {
	printf("%s [%3d]: ", prompt, *value);
	pi_readline(line, sizeof(line));
	if (sscanf(line, "%d", &new_value) != 1) {
	    printf("No number entered.\n");
	    return;
	}
    }

    if (new_value < min || new_value > max) {
	printf("%d is out of range %d..%d\n", new_value, min, max);
    } else {
	*value = new_value;
	printf("%s = %d\n", prompt, *value);
    }
}

static void read_pin(bool validate_only, const char *prompt, int *value) {
    read_int(validate_only, prompt, value, 0, 29);
}

static void read_bool(bool validate_only, const char *prompt, int *value) {
    read_int(validate_only, prompt, value, 0, 1);
}

static void set_lane_config(int id, lane_config_t *c, bool validate_only = false) {
    printf("Lane %d configuration:\n", id);
    printf("----------------------\n\n");
    read_pin (validate_only, "  Filament present pin", &c->present);
    read_pin (validate_only, "   Filament loaded pin", &c->loaded);
    read_pin (validate_only, "          Motor EN pin", &c->enable);
    read_pin (validate_only, "         Motor DIR pin", &c->dir);
    read_pin (validate_only, "        Motor STEP pin", &c->step);
    read_pin (validate_only, "  TMC2209 UART Address", &c->uart_address);
    read_bool(validate_only, "Invert motor direction", &c->invert);
}

static void set_motor_config(motor_config_t *c, bool validate_only = false) {
    printf("General Motor Configuration:\n");
    printf("----------------------------\n\n");
    read_pin(validate_only, "                    UART Tx pin", &c->tx);
    read_pin(validate_only, "                    UART Rx pin", &c->rx);
    read_int(validate_only, "               RMS current (mA)", &c->rms_current, 0, 1500);
    read_int(validate_only, "                  Microstepping", &c->microstepping, 1, 256);
    read_int(validate_only, "                       Steps/mm",  &c->steps_per_mm, 1);
    read_int(validate_only, "      Preloading speed (mm/sec)", &c->preload_speed, 1, 100);
    read_int(validate_only, "         Loading speed (mm/sec)", &c->loading_speed, 1, 100);
    read_int(validate_only, "Buffer refilling speed (mm/sec)", &c->refill_speed, 1, 100);
}

static void set_buffer_config(buffer_config_t *c, bool validate_only = false) {
    printf("Filament Buffer Configuration:\n");
    printf("------------------------------\n\n");
    read_pin(validate_only, "  Input pin", &c->input);
    read_pin(validate_only, "   Full pin", &c->full);
    read_pin(validate_only, "  Empty pin", &c->empty);
}

static void set_error_config(error_config_t *c, bool validate_only = false) {
    read_int(validate_only, "              RGB Pin", &c->rgb);
    read_int(validate_only, "           MM to load", &c->mm_to_load);
    read_int(validate_only, "          MM to retry", &c->mm_to_retry);
    read_int(validate_only, "          MM to load2", &c->mm_to_load2);
    read_int(validate_only, "y output timeout (us)", &c->y_output_timeout_us);
    read_int(validate_only, "y output retract (mm)", &c->y_output_retract_mm);
}

static void set_all_config(config_t *c, bool validate_only = false) {
    printf("\n");
    set_lane_config(1, &c->lanes[0], validate_only);
    printf("\n");
    set_lane_config(2, &c->lanes[1], validate_only);
    printf("\n");
    set_motor_config(&c->motor_config, validate_only);
    printf("\n");
    set_buffer_config(&c->buffer, validate_only);
    printf("\n");
    set_error_config(&c->error, validate_only);
    if (! validate_only && storage) storage->save(c);
}

// -------------------------- END CONFIG --------------------------

#define TRACE_STATE(x) printf("%s => %s\n", name, x)

class Switch {
public:
    Switch(int pin, ThreadInterruptNotifier *notifier, bool is_inverted = false) : pin(pin) {
	input = new GPInput(pin);
	input->set_pullup_up();
	input->set_debounce_us(1000);
	input->set_is_inverted(is_inverted);
	update();
	input->set_notifier(notifier);
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
    Lane(LaneSwitches *lane_switches, BufferSwitches *buffer_switches, Stepper *stepper, LaneLight *light, const char *name) : name(name), lane_switches(lane_switches), buffer_switches(buffer_switches), stepper(stepper), light(light) {
    }

    us_time_t update() {
	bool is_present = lane_switches->is_present();
        bool is_loaded = lane_switches->is_loaded();
	bool buffer_is_full = buffer_switches->buffer_is_full();
	bool buffer_is_empty = buffer_switches->buffer_is_empty();
	bool has_y_output = buffer_switches->has_y_output();

	int feed;
	enum State old_state;
	us_time_t polling_us;

	do {
	    feed = 0;
	    old_state = state;
	    polling_us = 0;

	    switch (state) {
	    case INIT:
		target_mm = stepper->get_mm_moved();
		if (is_loaded && has_y_output) state = ACTIVE;
		else if (is_loaded) state = ACTIVATING;
		else state = EMPTY;
		printf("%s: initial state: %s\n", name, state_to_string(state));
		break;

	    case EMPTY:
		light->empty();
		if (is_present) {
		    preloading_started_at = us_now();
		    state = PRE_LOADING;
		}
		break;
	    case PRE_LOADING:
		light->preloading();
		if (! is_present) state = EMPTY;
		else if (is_loaded) state = PRE_LOADING_RETRACT;
		else if (us_elapsed_ms_now(&preloading_started_at) > 30*1000) state = PRE_LOADING_ERROR;
		else {
		    feed = config.motor_config.preload_speed;
		    polling_us = 1000*1000;
		}
		break;
	    case PRE_LOADING_RETRACT:
		if (! is_present) state = EMPTY;
		else if (! is_loaded) state = READY;
		else feed = -config.motor_config.preload_speed;
		break;
	    case PRE_LOADING_ERROR:
		light->error();
		if (! is_present) state = EMPTY;
		break;
	    case READY:
		light->ready();
		break;
	    case ACTIVATING:
		if (is_loaded) {
		    state = LOADING;
		    stepper->reset_mm_moved();
		    target_mm = config.error.mm_to_load;
		} else {
		    feed = config.motor_config.loading_speed;
		}
		break;
	    case LOADING:
		light->loading();
		// TODO: if (buffer_is_empty) really short filament in here somewhere??
		if (! is_loaded && ! has_y_output) {
		    state = EMPTY;
		} else if (has_y_output) {
		    state = ACTIVE;
		    y_output_target_mm = target_mm = stepper->get_mm_moved() + 60;
		    wait_until = us_now() + config.error.y_output_timeout_us;
		} else if (buffer_is_full || stepper->get_mm_moved() >= target_mm) {
		    target_mm = stepper->get_mm_moved() - config.error.mm_to_retry;
		    state = LOADING_RETRACT;
		    n_buffer_retries++;
		} else {
		    feed = config.motor_config.loading_speed;
		    polling_us = 500*1000;
		}
		break;
	    case LOADING_RETRACT:
		light->error();
		if (! is_loaded) {
		    target_mm = stepper->get_mm_moved() + config.error.mm_to_load;
		    state = LOADING;
		} else if (stepper->get_mm_moved() <= target_mm) {
		    target_mm = stepper->get_mm_moved() + config.error.mm_to_retry + config.error.mm_to_load2;
		    state = LOADING;
		} else {
		    feed = -config.motor_config.loading_speed;
		    polling_us = 100*1000;
		}
		break;
	    case ACTIVE:
		light->active();
		if (buffer_is_full && buffer_is_empty) state = ERROR;
		else if (! is_loaded) state = EMPTYING;
		else if (buffer_is_full) state = WAITING;
		else feed = config.motor_config.refill_speed;
		break;
	    case WAITING: {
		light->waiting();
		us_time_t now = us_now();
		if (! is_loaded) {
		    state = EMPTYING;
		} else if (buffer_is_empty) {
		    state = ACTIVE;
		} else if (stepper->get_mm_moved() < target_mm) {
		    if (now >= wait_until) {
			target_mm = stepper->get_mm_moved() - config.error.y_output_retract_mm;
			state = ACTIVE_RETRACT;
			n_y_retries++;
		    } else {
			polling_us = wait_until - now;
		    }
		}
		break;
	    }
	    case ACTIVE_RETRACT:
		light->error();
		if (stepper->get_mm_moved() <= target_mm) {
		    state = ACTIVE;
		    wait_until = us_now() + config.error.y_output_timeout_us;
		    target_mm = y_output_target_mm;
		} else {
		    feed = -config.motor_config.loading_speed;
		    polling_us = 100*1000;
		}
		break;
	    case EMPTYING:
		light->emptying();
		if (! has_y_output) state = EMPTY;
		break;
	    case ERROR:
		light->error();
		break;
	    case STOP:
		light->stop();
		break;
	    case RETRACT:
		light->retract();
		if (! is_present) state = STOP;
		else feed = config.retract.speed;
		break;
	    }

	    trace_state(old_state);
        } while (state != old_state);

	stepper->set_speed(feed);
	return polling_us;
    }

public:
    bool is_active() {
	return state > READY && state < STOP;
    }

    bool is_ready() {
	return state == READY;
    }

    us_time_t emptying() {
	state = EMPTYING;
	return update();
    }

    us_time_t activate() {
	assert (state == READY);
	state = ACTIVATING;
	return update();
    }

    void dump_state() {
	printf("%s: %s", name, state_to_string(state));
	if (n_buffer_retries) printf(" || %d buffer retries", n_buffer_retries);
	if (n_y_retries) printf(" || %d y retries", n_y_retries);
	if (wait_until > us_now()) printf(" [wait %d ms]", (int) (wait_until - us_now()) / 1000);
	lane_switches->dump_state();
	if (is_active()) buffer_switches->dump_state();
	printf(" << ");
	stepper->dump_state();
	printf(" >>");
	printf(" target_mm %.2f", target_mm);
    }

    void error() {
	stepper->set_speed(0);
	change_state(STOP);
	state = STOP;
    }

    void stop() {
	change_state(STOP);
    }

    void resume() {
	change_state(INIT);
    }

    void retract() {
	change_state(RETRACT);
    }

private:
    const char *name;
    LaneSwitches *lane_switches;
    BufferSwitches *buffer_switches;
    Stepper *stepper;
    LaneLight *light;

    us_time_t preloading_started_at = 0;
    double target_mm = 0, y_output_target_mm = 0;
    us_time_t wait_until = 0;
    int n_buffer_retries = 0, n_y_retries = 0;

    enum State {
	    INIT,
	    EMPTY, PRE_LOADING, PRE_LOADING_RETRACT, PRE_LOADING_ERROR, READY,
	    ACTIVATING, LOADING, LOADING_RETRACT,
	    ACTIVE, WAITING, ACTIVE_RETRACT,
	    EMPTYING,
	    ERROR,
	    STOP, RETRACT
	} state = INIT;

private:
    void change_state(enum State new_state) {
	enum State old_state = state;
	state = new_state;
	trace_state(old_state);
    }

    const char *state_to_string(enum State state) {
	switch (state) {
	case INIT: return "init";
	case EMPTY: return "empty";
	case PRE_LOADING: return "pre-loading";
	case PRE_LOADING_RETRACT: return "pre-loading(retract)";
	case PRE_LOADING_ERROR: return "pre-loading(error)";
	case READY: return "ready";
	case ACTIVATING: return "activating";
	case LOADING: return "loading";
	case LOADING_RETRACT: return "loading(retract)";
	case ACTIVE: return "active";
	case WAITING: return "waiting";
	case ACTIVE_RETRACT: return "active(retract)";
	case EMPTYING: return "emptying";
	case ERROR: return "error";
	case STOP: return "stop";
	case RETRACT: return "retract";
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
    tmc->set_microstepping(config.motor_config.microstepping);
    tmc->set_rms_current(config.motor_config.rms_current);
}

static Stepper *create_lane_stepper(lane_config_t *config, motor_config_t *motor_config, const char *name, UART_Tx *tx) {
    Output *enable = new GPOutput(config->enable);
    Output *dir = new GPOutput(config->dir);
    Output *step = new GPOutput(config->step);
    dir->set_is_inverted(config->invert);
    configure_tmc(tx, config->uart_address);
    Stepper *stepper = new Stepper(enable, dir, step, name);
    stepper->set_steps_per_mm(motor_config->steps_per_mm * motor_config->microstepping / 16);
    return stepper;
}

class ThreadCondNotifier : public ThreadInterruptNotifier {
public:
    ThreadCondNotifier(PiMutex *lock, PiCond *cond, const char *name = "notifier") : ThreadInterruptNotifier(name), lock(lock), cond(cond) {
    }

    void on_change_safe(void) override {
	lock->lock();
	cond->broadcast();
	lock->unlock();
    }

private:
    PiMutex *lock;
    PiCond *cond;
};

static const char *lane_names[2] = { "lane-1", "lane-2" };
static const char *stepper_names[2] = { "stepper-1", "stepper-2" };

class Coordinator : public PiThread {
public:
    Coordinator(Lights *lights) : PiThread("coordinator"), lights(lights) {
	lock = new PiMutex();
	cond = new PiCond();

	lock->lock();
	notifier = new ThreadCondNotifier(lock, cond);

	tx = pico_new_pio_uart_tx(config.motor_config.tx, 115200);
	buffer_switches = new BufferSwitches(&config.buffer, notifier);
	for (int i = 0; i < 2; i++) {
	    lane_switches[i] = new LaneSwitches(&config.lanes[i], notifier);
	    lane[i] = new Lane(lane_switches[i], buffer_switches, create_lane_stepper(&config.lanes[i], &config.motor_config, stepper_names[i], tx), new LaneLight(lights, i), lane_names[i]);
	}

	manual_switch = new Switch(config.retract.manual, notifier);
	retract_switches[0] = new Switch(config.retract.lane[0], notifier, true);
	retract_switches[1] = new Switch(config.retract.lane[1], notifier, true);
	retract_start = us_now();

	if (manual_switch->get()) {
	    stop();
	} else if (! lane_switches[0]->is_loaded() && ! lane_switches[1]->is_loaded() && buffer_switches->has_y_output()) {
	    // Special case, neither lane is active but one of them was active
	    // when we last stopped running.  Since we don't know which one is
	    // supposed to active, move them both to emptying
	    lane[0]->emptying();
	    lane[1]->emptying();
	}

	update(true);

	lock->unlock();

	// TODO: What to do here!?!?!
	while (lane[0]->is_active() && lane[1]->is_active()) {
	    lane[0]->error();
	    lane[1]->error();
	    printf("\n\nFATAL ERROR: both lanes think they are active!!\n");
	    ms_sleep(1000);
	}

	start();
    }

    void main(void) override {
	lock->lock();
	while (1) {
	    if (polling_us) cond->wait_for(lock, polling_us);
	    else cond->wait(lock);
	    polling_us = 0;
	    update();
	}
    }

    void update(bool force = true) {
	bool output_changed = buffer_switches->update();
	for (int i = 0; i < 2; i++ ){
	    bool changed = lane_switches[i]->update();
	    if (force || changed || (active_lane == lane[i] && output_changed)) update_polling_us(lane[i]->update());
	}

	if (active_lane && ! active_lane->is_active()) active_lane = NULL;

	if (manual_switch->update()) {
	    if (manual_switch->get()) stop();
	    else resume();
	}

	for (int i = 0; i < 2; i++) {
	    if (retract_switches[i]->update() && manual_switch->get()) {
		if (retract_switches[i]->get()) {
		    lane[i]->retract();
		    retract_start = us_now();
		} else if (us_now() - retract_start <= RETRACT_ON_AFTER_US) {
		    lane[i]->stop();
		}
		update(true);
	    }
	}

	if (! active_lane) {
	    update_activate();

	    if (active_lane && ! active_lane->is_active()) {
		update_polling_us(active_lane->activate());
		printf("activated: ");
		active_lane->dump_state();
		printf("\n");
	    }
	}
    }

    void dump_state() {
	printf("======== Current State ===============\n");
	for (int i = 0; i < 2; i++) {
	    if (active_lane) printf("%s: ", lane[i] == active_lane ? "ACTIVE" : "      ");
	    lane[i]->dump_state();
	    printf("\n");
	}
	printf("all switches: lane_1:");
	lane_switches[0]->dump_state();
	printf(" || lane_2:");
	lane_switches[1]->dump_state();
	printf(" || output:");
	buffer_switches->dump_state();
	printf(" || manual %d retract %d %d\n", manual_switch->get(), retract_switches[0]->get(), retract_switches[1]->get());
	printf("\n");
    }

    void stop() {
	for (int i = 0; i < 2; i++) lane[i]->stop();
	update(true);
    }

    void resume() {
	for (int i = 0; i < 2; i++) lane[i]->resume();
	update(true);
    }

private:
    void update_polling_us(us_time_t new_polling_us) {
	if (! new_polling_us) return;
	else if (polling_us == 0 || new_polling_us < polling_us) polling_us = new_polling_us;
    }

    void update_activate() {
	for (int i = 0; i < 2; i++) {
	    if (lane[i]->is_active()) {
		active_lane = lane[i];
		return;
	    }
	}
	for (int i = 0; i < 2; i++) {
	    if (lane[i]->is_ready()) {
		active_lane = lane[i];
		return;
	    }
	}
    }

private:
    Lights *lights;

    PiMutex *lock;
    PiCond *cond;
    ThreadCondNotifier *notifier;

    UART_Tx *tx;
    BufferSwitches *buffer_switches;
    LaneSwitches *lane_switches[2];
    Lane *lane[2];

    Switch *manual_switch;
    Switch *retract_switches[2];
    us_time_t retract_start;

    Lane *active_lane = NULL;
    us_time_t polling_us = 0;
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

    boot_at = us_now();

    Lights *lights = new Lights();

    if (PersistentStorage::exists()) {
	printf("FRAM exists, using it for persistent storage\n");
	storage = new PersistentStorage();
	if (! storage->load(&config)) printf("FAILED to load existing configuration\n");
    } else {
	printf("WARNING: no FRAM detected, storage is disabled and using factory config\n");
    }

    coordinator = new Coordinator(lights);
    StateDumper *state_dumper = new StateDumper(coordinator);

    printf("Created coordinator, entering interactive loop.\n");
    while (1) {
	static char line[1024];

	if (pi_readline(line, sizeof(line)) != NULL) {
	    if (strcmp(line, "bootsel") == 0) {
		printf("Rebooting to bootloader.\n"); fflush(stdout);
		pi_reboot_bootloader();
	    } else if (strcmp(line, "reboot") == 0) {
		printf("Rebooting.\n"); fflush(stdout);
		pi_reboot();
	    } else if (strcmp(line, "state") == 0) {
		coordinator->dump_state();
	    } else if (strcmp(line, "threads") == 0 || strcmp(line, "t") == 0) {
		pi_threads_dump_state();
		printf("%d bytes free memory.\n", pi_threads_get_free_ram());
	    } else if (strncmp(line, "state-dumper", 12) == 0 || strcmp(line, "s") == 0) {
		int enabled = true;
		sscanf(&line[12], "%d", &enabled);
		if (enabled) state_dumper->enable();
		else state_dumper->disable();
	    } else if (strcmp(line, "factory-reset") == 0 && storage) {
		if (storage) storage->save(&factory_config);
	    } else if (strcmp(line, "show") == 0 || strcmp(line, "config") == 0) {
		set_all_config(&config, true);
	    } else if (strncmp(line, "set ", 4) == 0 || strncmp(line, "show ", 5) == 0) {
		bool validate_only = strncmp(line, "show ", 5) == 0;
		const char *what = &line[validate_only ? 5 : 4];
		bool dirty = ! validate_only;

		if (strcmp(what, "all") == 0) {
		    set_all_config(&config, validate_only);
		    dirty = false;
		} else if (strcmp(what, "lane_1") == 0) {
		    set_lane_config(1, &config.lanes[0], validate_only);
		} else if (strcmp(what, "lane_2") == 0) {
		    set_lane_config(2, &config.lanes[1], validate_only);
		} else if (strcmp(what, "motor") == 0) {
		    set_motor_config(&config.motor_config, validate_only);
		} else if (strcmp(what, "buffer") == 0) {
		    set_buffer_config(&config.buffer, validate_only);
		} else {
		    printf("Invalid configuration category: '%s'\n", what);
		    dirty = false;
		}
		if (dirty) {
		    if (storage) storage->save(&config);
		}
	    } else if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
		printf("\nConfiguration:");
		printf("\n--------------\n");
		printf("set (all | lane_1 | lane_2 | motor | buffer): modify configuration\n");
		printf("show [all | lane_1 | lane_2 | motor | buffer]: show configuration\n");
		printf("config : show full configuration\n");
		if (storage) printf("factory-reset\n");
		printf("\nPico Control:");
		printf("\n-------------\n");
		printf("bootsel: reboot to bootloader mode\n");
		printf("reboot\n");
		printf("state: dump state\n");
		printf("state-dumper [0|1]\n");
		printf("threads: dump thread state\n");
	    } else if (line[0]) {
		printf("help or ? for usage\n");
	    }
	}
    }
}

int main(int argc, char **argv) {
    pi_init_with_threads(threads_main, argc, argv);
    return 0;
}
