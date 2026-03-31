#include "pi.h"
#include <string.h>
#include "commands.h"
#include "fram-mb85c.h"
#include "httpd-server.h"
#include "stdin-reader.h"
#include "stdout-writer.h"
#include "threads-console.h"
#include "uart-channel.h"
#include "pi-threads.h"
#include "wifi.h"

#include "index.html.h"
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

static class Storage *storage;

class Storage {
public:
    Storage() {
        i2c_init_bus(I2C_BUS, I2C_SDA, I2C_SCL);
        fram = new FRAM_MB85C(I2C_BUS);
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
};

#ifndef PLATFORM_linux
class Channel : public UARTChannel {
public:
    Channel(int tx_pin, int rx_pin) : UARTChannel(tx_pin, rx_pin) {
    }

    void on_command(int _cmd, const void *data, int n_data) override {
	cmd_t cmd = (cmd_t) _cmd;
	switch(cmd) {
	case CMD_PING: send_command(CMD_PONG); break;
	case CMD_PONG: printf("got my pong\n"); break;
	}
    }
};
#endif

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

    HttpdResponse *open() {
	return new HttpdResponse(new MemoryBuffer(data, len));
    }

private:
    const uint8_t *data;
    size_t len;
};

class HttpdHandler : public HttpdSubstitutionHandler {
public:
    HttpdHandler(HttpdFilenameHandler *base) : HttpdSubstitutionHandler(base) {
    }

    const char *get_value_of(const char *key) {
	if (strcmp(key, "LANE1_DATA") == 0) {
	    return "{ i1: 'input-1', i2: 42 }";
	}
	return NULL;
    }
};

static void threads_main(int argc, char **argv) {
    ms_sleep(1000);

    printf("Creating a console\n");
    new HttpdConsole();

    printf("Creating Storage\n");
    storage = new Storage();
    if (storage->load_httpd_config(&httpd_config)) printf("Loading previous config\n");
    else printf("Failed to load previous config\n");

#ifndef PLATFORM_linux
    printf("Creating channel to the SKR Pico\n");
    new Channel(0, 1);
#endif

    printf("Starting WiFi\n");
    if (httpd_config.ap.ssid[0]) wifi_set_ap(httpd_config.ap.ssid, httpd_config.ap.password);
    wifi_init("infinite-feeder");
    wifi_wait_for_connection();

    printf("Creating Httpd server\n");
    auto httpd = new HttpdServer(httpd_config.port);
    httpd->add_file_handler("/", new HttpdRedirectHandler("/index.html"));
    httpd->add_file_handler("/index.html", new HttpdHandler(new StaticHandler(index_html, index_html_len)));
    httpd->add_file_handler("/infinite-feeder.css", new StaticHandler(infinite_feeder_css, infinite_feeder_css_len));
    httpd->add_file_handler("/favicon.ico", new StaticHandler(favicon_ico, favicon_ico_len));

    printf("Starting Httpd server on port %d\n", httpd_config.port);
    httpd->start();
}

int main(int argc, char **argv) {
    pi_init_with_threads(threads_main, argc, argv);
}

