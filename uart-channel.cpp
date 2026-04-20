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
    lock = new PiMutex();
}

void UARTChannel::main(void) {
    while (1) {
	uint8_t last_three[3];
	last_three[0] = rx->getc();
	last_three[1] = rx->getc();
	last_three[2] = rx->getc();
	while (last_three[0] != 0xff || last_three[1] != 0 || last_three[2] != 0x7f) {
	    printf("uart-channel: rejected %x %x %x\n", last_three[0], last_three[1], last_three[2]);
	    last_three[0] = last_three[1];
	    last_three[1] = last_three[2];
	    last_three[2] = rx->getc();
	}

printf("got preamble\n");
	int cmd, n_data;

	read_int(&cmd);
printf("cmd=%d\n", cmd);
	read_int(&n_data);
printf("n_data=%d\n", n_data);

	void *data = NULL;

	if (n_data != 0) {
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

printf("on_command %d %p %d\n", cmd, data, n_data);
	PiCond *notify_cond = on_command(cmd, data, n_data);

printf("free data %p\n", data);
	if (data) fatal_free(data);

printf("uart_channel: cmd=%d notifies %p\n", cmd, notify_cond);
	if (notify_cond) {
printf("lock\n");
	    lock->lock();
printf("broadcast\n");
	    notify_cond->broadcast();
	    lock->unlock();
printf("unlock\n");
	}
printf("uart_channel: done %d\n", cmd);
    }
}

bool UARTChannel::send_command(int cmd, const void *data, int n_data, PiCond *wait_cond, int timeout_ms, int max_retries) {
    int retries = 0;

printf("lock\n");
    lock->lock();

    do {
	if (max_retries >= 0 && retries++ > max_retries) {
printf("uart_channel: failed to get a reply in %d retries\n", max_retries);
	    lock->unlock();
printf("unlock\n");
	    return false;
	}
printf("uart_channel: send cmd=%d, n=%d, cond=%p, ms=%d, retries=%d/%d\n", cmd, n_data, wait_cond, timeout_ms, retries, max_retries);
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
printf("sent wait_for %p %d\n", wait_cond, timeout_ms);
    } while (wait_cond && ! wait_cond->wait_for(lock, timeout_ms * 1000));

    lock->unlock();
printf("unlock\n");
    return true;
}

void UARTChannel::write_int(int value) {
    tx->write(&value, sizeof(value));
}

void UARTChannel::read_int(int *value) {
    rx->read(value, sizeof(*value));
}

#endif
