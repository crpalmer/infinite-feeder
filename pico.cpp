#include "pi.h"
#include <string.h>
#include "commands.h"
#include "uart-channel.h"
#include "pi-threads.h"

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

static void threads_main(int argc, char **argv) {
    ms_sleep(1000);
    printf("pico starting\n");

    new Channel(0, 1);
}

int main(int argc, char **argv) {
    pi_init_with_threads(threads_main, argc, argv);
}

