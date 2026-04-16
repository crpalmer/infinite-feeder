#ifndef PLATFORM_linux

#include "pi.h"
#include "mem.h"
#include "uart-channel.h"

UARTChannel::UARTChannel(int tx_pin, int rx_pin, const char *name) : PiThread(name) {
    tx = pico_new_uart_tx(tx_pin);
    rx = pico_new_uart_rx(rx_pin);
    start();
}

void UARTChannel::main(void) {
    while (1) {
	uint8_t last_three[3] = { 1, 1, 1 };
	for (;;) {
	    last_three[0] = last_three[1];
	    last_three[1] = last_three[2];
	    last_three[2] = rx->getc();
	    if (last_three[0] == 0xff && last_three[1] == 0 && last_three[2] == 0x7f) break;
	    printf("uart-channel: rejected %x %x %x\n", last_three[0], last_three[1], last_three[2]);
	}

	int cmd, n_data;
	void *data = NULL;

	if (! read_int(&cmd) || ! read_int(&n_data) || n_data < 0) continue;

	if (n_data > 0) {
	    data = fatal_malloc(n_data);
	    rx->read(data, n_data);
	}

	on_command(cmd, data, n_data);

	if (data) fatal_free(data);
    }
}

void UARTChannel::send_command(int cmd, const void *data, int n_data) {
    tx->putc(0xff);
    tx->putc(0);
    tx->putc(0x7f);
    write_int(cmd);
    write_int(n_data);
    tx->write(data, n_data);
}

void UARTChannel::write_int(int value) {
    char str[20];
    sprintf(str, "%d\n", value);
    tx->write(str);
}

bool UARTChannel::read_int(int *value) {
    *value = 0;
    while (1) {
	char c = rx->getc();
	if (isdigit(c)) (*value) = (*value)*10 + (c - '0');
	else if (c == '\n') return true;
	else return false;
    }
}
#endif
