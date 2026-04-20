#ifndef __UART_CHANNEL_H__
#define __UART_CHANNEL_H__

#ifndef PLATFORM_linux

#include "pi-threads.h"
#include "uart.h"

class UARTChannel : public PiThread {
public:
    UARTChannel(int tx_pin, int rx_pin, const char *name = "uart-channel");
    void main(void) override;

    bool send_command(int cmd, const void *payload = NULL, int n_payload = 0, PiCond *wait_cond = NULL, int timeout_ms = 1000, int max_retries = -1);
    bool send_command(int cmd, PiCond *wait_cond, int timeout_ms = 1000, int max_retries = -1) { return send_command(cmd, NULL, 0, wait_cond, timeout_ms, max_retries); }
    virtual PiCond *on_command(int cmd, const void *payload, int n_payload) = 0;

    virtual void data_range(int cmd, int *low, int *high) {
	*low = 0;
	*high = 1<<16;
    }

private:
    void write_int(int value);
    void read_int(int *value);

    UART_Tx *tx;
    UART_Rx *rx;

    PiMutex *lock;
};

#endif

#endif
