#include "pi.h"
#include <string.h>
#include "commands.h"
#include "httpd-server.h"
#include "stdin-reader.h"
#include "stdout-writer.h"
#include "threads-console.h"
#include "uart-channel.h"
#include "pi-threads.h"
#include "wifi.h"

#include "index.html.h"
#include "infinite-feeder.css.h"

static struct {
    char hostname[128];
    int port;
    struct {
        char ssid[128];
        char password[32];
    } ap;
} httpd_config = {
    "infinite-feeder",
    80,
    { "", "" }
};

static void save_httpd_state() {
    printf("Saving my state\n");
}

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
	} else if (is_command(cmd, "ssid")) {
	    process_setting(arg, "ssid", httpd_config.ap.ssid, sizeof(httpd_config.ap.ssid));
	    if (httpd_config.ap.ssid[0]) wifi_set_ap(httpd_config.ap.ssid, httpd_config.ap.password);
	} else {
	    ThreadsConsole::process_cmd(cmd);
	}
    }

    void usage(void) override {
	ThreadsConsole::usage();
	printf("hostname [hostname]\n");
	printf("password [password]\n");
	printf("ssid [ssid]\n");
    }

private:
    void process_setting(const char *arg, const char *name, char *value, size_t len) {
	if (! *arg) {
	    printf("Current %s: %s\n", name, value);
	} else if (*arg && strlen(arg) < len) {
	    strcpy(value, arg);
	    save_httpd_state();
	} else {
	    printf("%s too long\n", name);
	}
    }
};

static void threads_main(int argc, char **argv) {
    ms_sleep(1000);

    printf("Creating a console\n");
    new HttpdConsole();

    printf("Starting WiFi\n");
    if (httpd_config.ap.ssid[0]) wifi_set_ap(httpd_config.ap.ssid, httpd_config.ap.password);
    wifi_init("infinite-feeder");
    wifi_wait_for_connection();

    printf("Creating channel to the SKR Pico\n");
    new Channel(0, 1);

    printf("Creating Httpd server\n");
    auto httpd = new HttpdServer(httpd_config.port);
    httpd->add_file_handler("/", new HttpdRedirectHandler("/index.html"));
    //httpd->add_file_handler("/index.html", new HttpdRedirectHandler("/finance/index.html"));
    //httpd->add_file_handler("/infinite-feeder.css", new HttpdRedirectHandler("/finance/index.html"));

    printf("Starting Httpd server on port %d\n", httpd_config.port);
    httpd->start();

    printf("main thread is now exiting\n");
}

int main(int argc, char **argv) {
    pi_init_with_threads(threads_main, argc, argv);
}

