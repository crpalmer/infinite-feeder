#ifndef PLATFORM_linux

#include "pi.h"
#include "mem.h"
#include "uart-channel.h"

static const uint32_t MOD_ADLER = 65521;

uint32_t adler32(const void *data_vp, size_t len) {
    uint32_t a = 1, b = 0;
    const uint8_t *data = (uint8_t *) data_vp;

    for (size_t index = 0; index < len; index++) {
        a = (a + data[index]) % MOD_ADLER;
        b = (b + a) % MOD_ADLER;
    }

    return (b << 16) | a;
}

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
	    int low, high;

	    data_range(cmd, &low, &high);
	    if (n_data < low || n_data > high) {
		printf("uart-channel: invalid data range: cmd=%d, valid=%d..%d, received=%d\n", cmd, low, high, n_data);
		continue;
	    }

	    uint32_t checksum;
	    data = fatal_malloc(n_data);
	    rx->read(data, n_data);
	    rx->read(&checksum, sizeof(checksum));
	    uint32_t received_checksum = adler32(data, n_data);
	    if (received_checksum != checksum) {
		printf("uart-channel: invalid checksum %x != %x\n", (unsigned) checksum, (unsigned) received_checksum);
		fatal_free(data);
		continue;
	    }
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
    if (n_data) {
	uint32_t checksum = adler32(data, n_data);
	tx->write(data, n_data);
	tx->write(&checksum, sizeof(checksum));
    }
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
