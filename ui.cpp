#include "pi.h"
#include <string.h>
#include <unordered_map>
#include "commands.h"
#include "config.h"
#include "fram-mb85c.h"
#include "httpd-server.h"
#include "pi-threads.h"
#include "status.h"
#include "stdin-reader.h"
#include "stdout-writer.h"
#include "threads-console.h"
#include "uart-channel.h"
#include "wifi.h"

#include "index.html.h"
#include "config.html.h"
#include "infinite-feeder.css.h"
#include "favicon.ico.h"

typedef struct {
    char hostname[128];
    int port;
    struct {
        char ssid[128];
        char password[32];
    } ap;
} httpd_config_t;

static httpd_config_t httpd_config = {
    "infinite-feeder",
    80,
    { "", "" }
};

static config_t config = factory_config;

static class Storage *storage;
static class Channel *channel;

static status_t status = empty_status;
static PiMutex *status_lock;

#ifdef PLATFORM_linux
#include "file.h"

class FakeFRAM : public FRAM {
public:
    FakeFRAM() {
	const char *name = "fake_fram.data";
	if (! file_exists(name)) {
	    if ((f = fopen(name, "w")) == NULL) perror(name);
	    else {
		for (size_t i = 0; i < get_capacity(); i++) fputc(' ', f);
		fclose(f);
	    }
	}
	f = fopen(name, "r+b");
	if (! f) {
	    perror(name);
	    exit(1);
	}
    }

    bool read(int offset, void *data, size_t n) override {
	if (fseek(f, offset, SEEK_SET) != 0) return false;
	if (fread(data, 1, n, f) != n) return false;
	return true;
    }

    bool write(int offset, const void *data, size_t n) override {
	if (fseek(f, offset, SEEK_SET) != 0) return false;
	if (fwrite(data, 1, n, f) != n) return false;
	fflush(f);
	return true;
    }

    size_t get_capacity() { return 128*1024; }

private:
    FILE *f;
};
#endif

#ifndef PLATFORM_linux
class Channel : public UARTChannel {
public:
    Channel(int tx_pin, int rx_pin) : UARTChannel(tx_pin, rx_pin) {
	lock = new PiMutex();
	status_cond = new PiCond();
	okay_cond = new PiCond();
    }

    void on_command(int _cmd, const void *data, int n_data) override {
	cmd_t cmd = (cmd_t) _cmd;
	switch(cmd) {
	case CMD_OKAY:
	    okay_cond->broadcast();
	    break;
	case CMD_PING: send_command(CMD_PONG); break;
	case CMD_PONG: printf("got my pong\n"); break;
	case CMD_STATUS:
	    if (n_data != sizeof(status)) {
		printf("channel: invalid status received (%d bytes when expecting %d bytes)\n", n_data, sizeof(status));
	    } else {
		status_lock->lock();
		memcpy(&status, data, n_data);
		status_lock->unlock();
		status_cond->broadcast();
	    }
	    break;
	case CMD_GET_CONFIG:
	    send_command(CMD_CONFIG_BLOB, &config, sizeof(config));
	    break;
	case CMD_STOP:
	case CMD_RESUME:
	case CMD_RETRACT:
	case CMD_CONFIG_BLOB:
	case CMD_GET_STATUS:
	case CMD_NEW_CONFIG_AVAILABLE:
	    printf("channel: received unexpected command %d\n", cmd);
	    break;
	}
    }

    void get_status() {
	send_synchronous_command(status_cond, CMD_GET_STATUS);
    }

    void send_config() {
	send_command(CMD_NEW_CONFIG_AVAILABLE);
    }

    void stop() {
	send_synchronous_command(okay_cond, CMD_STOP);
    }

    void resume() {
	send_synchronous_command(okay_cond, CMD_RESUME);
    }

    void retract(int lane) {
	send_synchronous_command(okay_cond, CMD_RETRACT, &lane, sizeof(lane));
    }

private:
    PiMutex *lock;
    PiCond *okay_cond;
    PiCond *status_cond;

    void send_synchronous_command(PiCond *cond, cmd_t cmd, void *data = NULL, int len = 0) {
	lock->lock();
	send_command(cmd, data, len);
	while (! status_cond->wait_for(lock, 1000*1000)) {
	    send_command(cmd, data, len);
	}
	lock->unlock();
    }
};
#else
class Channel {
public:
    Channel(int tx, int rx) {
	boot_at = us_now();
    }

    void get_status() {
        status.uptime = us_elapsed_ms_now(&boot_at) / 1000;
    }

    void send_config() {}

    void stop() {
	status.active_lane = -1;
	strcpy(status.lanes[0].state, "STOP");
	strcpy(status.lanes[1].state, "STOP");
    }

    void resume() {
	status.active_lane = 0;
	strcpy(status.lanes[0].state, "ACTIVE");
	strcpy(status.lanes[1].state, "READY");
    }

    void retract(int lane) {
	strcpy(status.lanes[lane].state, "RETRACT");
    }

private:
    us_time_t boot_at;
};
#endif

class Storage {
public:
    Storage() {
#ifdef PLATFORM_linux
	fram = new FakeFRAM();
#else
        i2c_init_bus(I2C_BUS, I2C_SDA, I2C_SCL);
        fram = new FRAM_MB85C(I2C_BUS);
#endif
    }

    bool load_httpd_config(httpd_config_t *c) {
	uint32_t magic;
        if (! fram->read(HTTPD_CONFIG_OFFSET, &magic, sizeof(magic))) return false;
        if (magic != HTTPD_CONFIG_MAGIC) return false;

	uint32_t version;
	if (! fram->read(HTTPD_CONFIG_VERSION_OFFSET, &version, sizeof(version))) return false;

	// TODO: Version upgrades
	if (version != HTTPD_CONFIG_VERSION) return false;

	if (! fram->read(HTTPD_CONFIG_DATA_OFFSET, c, sizeof(*c))) return false;
        return true;
    }

    bool save_httpd_config(httpd_config_t *c) {
	uint32_t magic = HTTPD_CONFIG_MAGIC;
	if (! fram->write(HTTPD_CONFIG_OFFSET, &magic, sizeof(magic))) return false;
	uint32_t version = HTTPD_CONFIG_VERSION;
	if (! fram->write(HTTPD_CONFIG_VERSION_OFFSET, &version, sizeof(version))) return false;
	return fram->write(HTTPD_CONFIG_DATA_OFFSET, c, sizeof(*c));
    }

    bool load_config(config_t *c) {
        uint32_t magic;
        if (! fram->read(CONFIG_OFFSET, &magic, sizeof(magic))) return false;
        if (magic != CONFIG_MAGIC) return false;

        uint32_t version;
        if (! fram->read(CONFIG_VERSION_OFFSET, &version, sizeof(version))) return false;
        //
        //
        // Upgrade old configurations

        if (version != CONFIG_VERSION) {
            printf("Loaded version %u of the configuration (should be %u)\n", (unsigned) version, (unsigned) CONFIG_VERSION);
            return false;
        }

        config_t new_c;
        if (! fram->read(CONFIG_DATA_OFFSET, &new_c, sizeof(new_c))) return false;
        *c = new_c;

        return true;
    }

    bool save_config(config_t *c) {
        uint32_t magic = CONFIG_MAGIC;
        if (! fram->write(CONFIG_OFFSET, &magic, sizeof(magic))) return false;
        uint32_t version = CONFIG_VERSION;
        if (! fram->write(CONFIG_VERSION_OFFSET, &version, sizeof(version))) return false;
        if (! fram->write(CONFIG_DATA_OFFSET, c, sizeof(*c))) return false;
	channel->send_config();
        return true;
    }

private:
    FRAM *fram;

    static const int I2C_BUS = 0;
    static const int I2C_SDA = 4;
    static const int I2C_SCL = 5;

    static const int HTTPD_CONFIG_OFFSET = 0;
    static const uint32_t HTTPD_CONFIG_MAGIC = 0x12345678;
    static const int HTTPD_CONFIG_VERSION_OFFSET = HTTPD_CONFIG_OFFSET + sizeof(HTTPD_CONFIG_MAGIC);
    static const uint32_t HTTPD_CONFIG_VERSION = 1;
    static const int HTTPD_CONFIG_DATA_OFFSET = HTTPD_CONFIG_VERSION_OFFSET + sizeof(HTTPD_CONFIG_VERSION);

    static const int CONFIG_OFFSET = 4096;
    static const uint32_t CONFIG_MAGIC = 0x87654321;
    static const int CONFIG_VERSION_OFFSET = CONFIG_OFFSET + sizeof(CONFIG_MAGIC);
    static const int CONFIG_DATA_OFFSET = CONFIG_VERSION_OFFSET + sizeof(CONFIG_VERSION);
};

class HttpdConsole : public PiThread, public ThreadsConsole {
public:
    HttpdConsole() : PiThread("console"), ThreadsConsole(new StdinReader(), new StdoutWriter()) { start(); }

    void main(void) { ThreadsConsole::main(); }

    void process_cmd(const char *cmd) override {
	const char *arg = cmd;
	while (*arg && ! isspace(*arg)) arg++;
	while (*arg && isspace(*arg)) arg++;

	if (is_command(cmd, "hostname")) {
	    process_setting(arg, "hostnmae", httpd_config.hostname, sizeof(httpd_config.hostname));
	} else if (is_command(cmd, "password")) {
	    process_setting(arg, "password", httpd_config.ap.password, sizeof(httpd_config.ap.password));
	    if (httpd_config.ap.ssid[0]) wifi_set_ap(httpd_config.ap.ssid, httpd_config.ap.password);
	} else if (is_command(cmd, "save")) {
	    if (storage->save_httpd_config(&httpd_config)) {
		printf("Saved configuration, rebooting.\n");
		pi_reboot();
	    } else {
		printf("Failed to save configuration.\n");
	    }
	} else if (is_command(cmd, "ssid")) {
	    process_setting(arg, "ssid", httpd_config.ap.ssid, sizeof(httpd_config.ap.ssid));
	    if (httpd_config.ap.ssid[0]) wifi_set_ap(httpd_config.ap.ssid, httpd_config.ap.password);
	} else {
	    ThreadsConsole::process_cmd(cmd);
	}
    }

    void usage(void) override {
	ThreadsConsole::usage();
	printf("\nConfiguration changes (do not take effect until you execute \"save\"\n\n");
	printf("hostname [hostname]\n");
	printf("password [password]\n");
	printf("ssid [ssid]\n");
	printf("\nsave : save configuration and reboot\n");
    }

private:
    void process_setting(const char *arg, const char *name, char *value, size_t len) {
	if (! *arg) {
	    printf("Current %s: %s\n", name, value);
	} else if (*arg && strlen(arg) < len) {
	    strcpy(value, arg);
	} else {
	    printf("%s too long\n", name);
	}
    }
};

class StaticHandler : public HttpdFilenameHandler {
public:
    StaticHandler(const uint8_t *data, size_t len) : data(data), len(len) {
    }

    HttpdResponse *open(HttpdRequest *request) override {
	return new HttpdResponse(new MemoryBuffer(data, len));
    }

private:
    const uint8_t *data;
    size_t len;
};

class StatusHandler : public HttpdSubstitutionHandler {
public:
    StatusHandler(HttpdFilenameHandler *base) : HttpdSubstitutionHandler(base) {
	for (int lane = 0; lane < 2; lane++) {
	    insert_lane_item(lane, "state", status.lanes[lane].state);
	    insert_lane_item(lane, "mm", &status.lanes[lane].mm);
	    insert_lane_item(lane, "present", &status.lanes[lane].present);
	    insert_lane_item(lane, "loaded", &status.lanes[lane].loaded);
	    bool present = status.lanes[lane].present;
	    bool loaded = status.lanes[0].loaded;
	    insert_lane_item(lane, "filament_state", present && loaded ? "present and loaded" : present ? "present" : loaded ? "loaded" : "empty");
	}
	imap["uptime"] = &status.uptime;
	imap["active_lane"] = &status.active_lane;
	imap["y_output"] = &status.y_output;
	imap["buffer_full"] = &status.buffer_full;
	imap["buffer_empty"] = &status.buffer_empty;
    }

    const char *get_value_of(const char *key) {
	if (smap[key]) {
	    return smap[key];
	}

	if (imap[key]) {
	    sprintf(tmp_string, "%d", *imap[key]);
	    return tmp_string;
	}
	return NULL;
    }

    HttpdResponse *open(HttpdRequest *request) override {
	const char *action_c;
	int action_len = request->get_parameter("action", &action_c);
	if (action_len > 0) {
	    std::string action = std::string(action_c, action_len);
	    if (action == "stop") channel->stop();
	    else if (action == "resume") channel->resume();
	    else if (action == "retract1") channel->retract(0);
	    else if (action == "retract2") channel->retract(1);
	}
	channel->get_status();
	return HttpdSubstitutionHandler::open(request);
    }

private:
    void insert_lane_item(int lane, const char *name, const int *ptr) {
	char full[10 + strlen(name)];
	sprintf(full, "lane%d_%s", lane+1, name);
	imap[full] = ptr;
    }

    void insert_lane_item(int lane, const char *name, const char *ptr) {
	char full[10 + strlen(name)];
	sprintf(full, "lane%d_%s", lane+1, name);
	smap[full] = ptr;
    }

    std::unordered_map<std::string, const char *> smap;
    std::unordered_map<std::string, const int *> imap;
    char tmp_string[50];
};

class ConfigHandler : public HttpdSubstitutionHandler {
public:
    ConfigHandler(HttpdFilenameHandler *base) : HttpdSubstitutionHandler(base) {
	for (int lane = 0; lane < 2; lane++) {
	    insert_lane_item(lane, "present",         &config.lanes[lane].present);
	    insert_lane_item(lane, "loaded",          &config.lanes[lane].loaded);
	    insert_lane_item(lane, "enable",          &config.lanes[lane].enable);
	    insert_lane_item(lane, "dir",             &config.lanes[lane].dir);
	    insert_lane_item(lane, "step",            &config.lanes[lane].step);
	    insert_lane_item(lane, "uart_address",    &config.lanes[lane].uart_address);
	    insert_lane_item(lane, "invert",          &config.lanes[lane].invert);
	}
	map["motor_rx"] =             &config.motor_config.rx;
	map["motor_tx"] =             &config.motor_config.tx;
	map["motor_rms_current"] =    &config.motor_config.rms_current;
	map["motor_microstepping"] =  &config.motor_config.microstepping;
	map["motor_steps_per_mm"] =   &config.motor_config.steps_per_mm;
	map["motor_preload_speed"] =  &config.motor_config.preload_speed;
	map["motor_loading_speed"] =  &config.motor_config.loading_speed;
	map["motor_refill_speed"] =   &config.motor_config.refill_speed;

	map["buffer_input"] = &config.buffer.input;
	map["buffer_full"] =  &config.buffer.full;
	map["buffer_empty"] = &config.buffer.empty;

	map["error_rgb"] =		   &config.error.rgb;
	map["error_mm_to_load"] =          &config.error.mm_to_load;
	map["error_mm_to_retry"] =         &config.error.mm_to_retry;
	map["error_mm_to_load2"] =         &config.error.mm_to_load2;
	map["error_y_output_timeout_us"] = &config.error.y_output_timeout_us;
	map["error_y_output_retract_mm"] = &config.error.y_output_retract_mm;
    }

    const char *get_value_of(const char *key) {
	if (map[key]) {
	    sprintf(tmp_string, "%d", *map[key]);
	    return tmp_string;
	}
	if (strcmp(key, "PIN_SELECT_OPTIONS") == 0) {
	    return
		"<option value=\"26\" :selected=\"value == 26\">THB</option>\n"
		"<option value=\"27\" :selected=\"value == 27\">TH0</option>\n"
		"<option value=\" 4\" :selected=\"value ==  4\">X-Stop</option>\n"
		"<option value=\" 3\" :selected=\"value ==  3\">Y-Stop</option>\n"
		"<option value=\"25\" :selected=\"value == 25\">Z-Stop</option>\n"
		"<option value=\"16\" :selected=\"value == 16\">E0-Stop</option>\n"
		"<option value=\"24\" :selected=\"value == 24\">RGB</option>\n"
		"<option value=\"29\" :selected=\"value == 29\">Servos</option>\n"
		"<option value=\"22\" :selected=\"value == 22\">Probe</option>\n"
		"<option value=\"17\" :selected=\"value == 17\">FAN1</option>\n"
		"<option value=\"18\" :selected=\"value == 18\">FAN2</option>\n"
		"<option value=\"20\" :selected=\"value == 20\">FAN3</option>\n"
		"<option value=\" 8\" :selected=\"value ==  8\">Motor UART TX</option>\n"
		"<option value=\" 9\" :selected=\"value ==  9\">Motor UART RX</option>\n"
		"<option value=\"12\" :selected=\"value == 12\">Motor X(EN)</option>\n"
		"<option value=\"11\" :selected=\"value == 11\">Motor X(STEP)</option>\n"
		"<option value=\"10\" :selected=\"value == 10\">Motor X(DIR)</option>\n"
		"<option value=\" 7\" :selected=\"value ==  7\">Motor Y(EN)</option>\n"
		"<option value=\" 6\" :selected=\"value ==  6\">Motor Y(STEP)</option>\n"
		"<option value=\" 5\" :selected=\"value ==  5\">Motor Y(DIR)</option>\n"
		"<option value=\" 2\" :selected=\"value ==  2\">Motor Z(EN)</option>\n"
		"<option value=\"19\" :selected=\"value == 19\">Motor Z(STEP)</option>\n"
		"<option value=\"28\" :selected=\"value == 28\">Motor Z(DIR)</option>\n"
		"<option value=\"15\" :selected=\"value == 15\">Motor E0(EN)</option>\n"
		"<option value=\"14\" :selected=\"value == 14\">Motor E0(STEP)</option>\n"
		"<option value=\"13\" :selected=\"value == 13\">Motor E0(DIR)</option>\n"
		"<option value=\"20\" :selected=\"value == 20\">Laser</option>\n"
		"<option value=\"23\" :selected=\"value == 23\">HE</option>\n"
		"<option value=\"21\" :selected=\"value == 21\">HB</option>\n"
	    ;
	}
	if (strcmp(key, "UART_ADDRESS_SELECT_OPTIONS") == 0) {
	    return
		"<option value=\"0\" :selected=\"value == 0\">X UART</option>\n"
		"<option value=\"2\" :selected=\"value == 2\">Y UART</option>\n"
		"<option value=\"1\" :selected=\"value == 1\">Z UART</option>\n"
		"<option value=\"3\" :selected=\"value == 3\">E0 UART</option>\n"
	    ;
	}
	return NULL;
    }

    HttpdResponse *open(HttpdRequest *request) override {
	bool save_needed = false;
	for (const auto& pair : map) {
	    const char *str;
	    int len = request->get_parameter(pair.first.c_str(), &str);
	    if (len >= 0) {
		char value[len+1];
		strncpy(value, str, len);
		value[len] = '\0';
		int int_value;
		if (sscanf(value, "%d", &int_value) == 1) {
		    if (*pair.second != int_value) save_needed = true;
		    *pair.second = int_value;
		}
	    }
	}
	if (save_needed) {
	    if (! storage->save_config(&config)) printf("*** FAILED TO SAVE CONFIGURATION ***\n");
	}
	return HttpdSubstitutionHandler::open(request);
    }

private:
    void insert_lane_item(int lane, const char *name, int *ptr) {
	char full[10 + strlen(name)];
	sprintf(full, "lane%d_%s", lane+1, name);
	map[full] = ptr;
    }

    std::unordered_map<std::string, int *> map;
    char tmp_string[50];
};

static void threads_main(int argc, char **argv) {
    ms_sleep(1000);

    printf("Starting WiFi connection (async)\n");
    if (httpd_config.ap.ssid[0]) wifi_set_ap(httpd_config.ap.ssid, httpd_config.ap.password);
    wifi_init("infinite-feeder");

    printf("Creating a console\n");
    new HttpdConsole();

    printf("Creating Storage\n");
    storage = new Storage();
    if (storage->load_httpd_config(&httpd_config)) printf("Loading previous HTTP config\n");
    else printf("Failed to load previous HTTP config\n");
    if (storage->load_config(&config)) printf("Loaded firmware config\n");
    else printf("Failed to load previous firmware config\n");

    status_lock = new PiMutex();

    printf("Creating channel to the SKR Pico\n");
    channel = new Channel(0, 1);

    printf("Creating Httpd server\n");
    auto httpd = new HttpdServer(httpd_config.port);
    httpd->add_file_handler("/", new HttpdRedirectHandler("/index.html"));
    httpd->add_file_handler("/index.html", new StatusHandler(new StaticHandler(index_html, index_html_len)));
    httpd->add_file_handler("/config.html", new ConfigHandler(new StaticHandler(config_html, config_html_len)));
    httpd->add_file_handler("/infinite-feeder.css", new StaticHandler(infinite_feeder_css, infinite_feeder_css_len));
    httpd->add_file_handler("/favicon.ico", new StaticHandler(favicon_ico, favicon_ico_len));

    printf("Waiting for WiFi connection to complete\n");
    wifi_wait_for_connection();

    printf("Starting Httpd server on port %d\n", httpd_config.port);
    httpd->start();
}

int main(int argc, char **argv) {
    pi_init_with_threads(threads_main, argc, argv);
}

