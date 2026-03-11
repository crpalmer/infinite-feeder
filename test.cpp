#include "pi.h"
#include "i2c.h"
#include "time-utils.h"

int main(int argc, char **argv) {
    pi_init();
    i2c_init_bus(0, 0, 1);
    ms_sleep(2000);
    assert(fd >= 0);
    size_t n = 128;
    uint16_t data[n];
    uint16_t data2[n];
    for (size_t i = 0; i < n; i++) data[i] = i;
    int addr = 0x50;
    int chunk = 16;
    int fd = i2c_open(0, addr);
    int ret;

    uint8_t *ptr = (uint8_t *)data;
    for (size_t start = 0; start < sizeof(data); start += chunk) {
ms_sleep(5);
	if ((ret = i2c_write(fd, start, &ptr[start], chunk)) != chunk+1) {
	    printf("0x%02x write @ %d failure: %d\n", addr, (int) start, ret);
	    goto done;
	}
    }

    ptr = (uint8_t *)data2;
    for (size_t start = 0; start < sizeof(data2); start += chunk) {
ms_sleep(5);
	if ((ret = i2c_read(fd, start, &ptr[start], chunk)) != chunk) {
	    printf("0x%02x read @ %d failure: %d\n", addr, (int) start, ret);
	    goto done;
	}
    }

    for (size_t i = 0; i < n; i++) {
	if (data2[i] != data[i]) printf("value of index %d failure %d != %d\n", i, data2[i], data[i]);
    }

    i2c_close(fd);
done:
    printf("Done.\n");
    while(1) {
    }
}
