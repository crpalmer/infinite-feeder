#ifndef __UART_CHANNEL_H__
#define __UART_CHANNEL_H__

#ifndef PLATFORM_linux

#include "pi-threads.h"
#include "uart.h"

class UARTChannel : public PiThread {
public:
    UARTChannel(int tx_pin, int rx_pin, const char *name = "uart-channel");
    void main(void) override;

    virtual void send_command(int cmd, const void *payload = NULL, int n_payload = 0);
    virtual void on_command(int cmd, const void *payload, int n_payload) = 0;

private:
    void write_int(int value);
    bool read_int(int *value);

private:
    UART_Tx *tx;
    UART_Rx *rx;
};

#endif

#endif
