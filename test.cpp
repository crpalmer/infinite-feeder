#include "pi.h"
#include "fram-at24c.h"
#include "i2c.h"
#include "time-utils.h"

static bool validate(uint16_t *d1, uint16_t *d2, size_t n) {
    int failures = 0;

    for (size_t i = 0; i < n; i++) {
	if (d1[i] != d2[i]) {
	    printf("Invalid value @ %d: %d != %d\n", (int) i, d1[i], d2[i]);
	    if (++failures > 10) return false;
	}
    }
    return failures == 0;
}

int main(int argc, char **argv) {
    pi_init();

    ms_sleep(2000);

    i2c_init_bus(0, 0, 1);
    FRAM *fram = new FRAM_AT24C(0);

    size_t n = 128;
    uint16_t data[n];
    uint16_t data2[n];

    for (size_t i = 0; i < n; i++) data[i] = i;

    if (! fram->write(0, data, sizeof(data))) {
	printf("write failure\n");
    } else if (! fram->read(0, data2, sizeof(data2))) {
	printf("read failure\n");
    } else if (! validate(data, data2, n)) {
	printf("validation failure\n");
    } else {
	printf("success\n");
    }

    while(1) {
    }
}
