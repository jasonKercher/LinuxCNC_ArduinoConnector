#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include <Wire.h>
#include <Adafruit_MCP23X17.h>

#define ssizeof(VAR_)    (ssize_t)sizeof(VAR_)
#define array_len_(ARR_) (ssizeof(ARR_) / ssizeof((ARR_)[0]))

typedef int8_t   i8;
typedef int16_t  i16;
typedef uint8_t  u8;
typedef uint16_t u16;

enum Expander_Port {
	P_A0,
	P_A1,
	P_A2,
	P_A3,
	P_A4,
	P_A5,
	P_A6,
	P_A7,
	P_B0,
	P_B1,
	P_B2,
	P_B3,
	P_B4,
	P_B5,
	P_B6,
	P_B7,
	P_COUNT
};

u8 EXPANDER_PINS[P_COUNT] = {
    21, /* A0 */
    22, /* A1 */
    23, /* A2 */
    24, /* A3 */
    25, /* A4 */
    26, /* A5 */
    27, /* A6 */
    28, /* A7 */
    1,  /* B0 */
    2,  /* B1 */
    3,  /* B2 */
    4,  /* B3 */
    5,  /* B4 */
    6,  /* B5 */
    7,  /* B6 */
    8,  /* B7 */
};

/* Define Expanders */
struct I2c_Bus {
	TwoWire* const wire;
	u8             addr;
	u8             isr;
};

/* TODO: FILL THIS IN */
#define EXANDERS 5
static const struct I2c_Bus I2C_BUS[EXANDERS] = {
    {&Wire1, 0x00, 0},
    {&Wire2, 0x00, 1},
    {&Wire2, 0x01, 1},
    {&Wire2, 0x02, 1},
    {&Wire2, 0x03, 1},
};
static Adafruit_MCP23X17 _expanders[EXANDERS];

/* Define inputs */
struct I2c_Io {
	u8 expander;
	u8 port;
};
#define ROWS 4
#define COLS 8

/* TODO: actually fill this out. */
static struct I2c_Io INPUTS[ROWS * COLS] = {
    {0, P_B0}, {0, P_B1}, {0, P_B2}, {0, P_B3}, {0, P_B4}, {0, P_B5}, {0, P_B6}, {0, P_B7},
    {0, P_A0}, {0, P_A1}, {0, P_A2}, {0, P_A3}, {0, P_A4}, {0, P_A5}, {1, P_B1}, {1, P_B3},
    {1, P_B5}, {1, P_B7}, {1, P_A0}, {1, P_A2}, {1, P_A4}, {1, P_A6}, {2, P_B0}, {2, P_B2},
    {2, P_B4}, {2, P_B6}, {2, P_A1}, {2, P_A3}, {2, P_A5}, {2, P_A7}, {3, P_B1}, {3, P_B3},
};

static struct I2c_Io OUTPUTS[ROWS * COLS] = {
    {1, P_B0}, {1, P_B1}, {1, P_B2}, {1, P_B3}, {1, P_B4}, {1, P_B5}, {1, P_B6}, {1, P_B7},
    {1, P_A0}, {1, P_A1}, {1, P_A2}, {1, P_A3}, {1, P_A4}, {1, P_A5}, {2, P_B1}, {2, P_B3},
    {2, P_B5}, {2, P_B7}, {2, P_A0}, {2, P_A2}, {2, P_A4}, {2, P_A6}, {3, P_B0}, {3, P_B2},
    {3, P_B4}, {3, P_B6}, {3, P_A1}, {3, P_A3}, {3, P_A5}, {3, P_A7}, {4, P_B1}, {4, P_B3},
};


/* TODO: Double check u8 is atomic */
static u8 _input_change;
void
i2c_input_isr() {
	_input_change = 1;
}

void
setup() {
	attachInterrupt(0, i2c_input_isr, CHANGE);
	attachInterrupt(1, i2c_input_isr, CHANGE);

	for (int i = 0; i < array_len_(_expanders); i += 1) {
		_expanders[i].enableAddrPins();
		_expanders[i].begin_I2C(MCP23XXX_ADDR | I2C_BUS[i].addr, I2C_BUS[i].wire);
	}

	for (int i = 0; i < array_len_(INPUTS); i += 1) {
		int idx = INPUTS[i].expander;
		int pin = EXPANDER_PINS[INPUTS[i].port];
		_expanders[idx].pinMode(pin, INPUT_PULLUP);
		_expanders[idx].setupInterruptPin(pin);
	}

	for (int i = 0; i < array_len_(OUTPUTS); i += 1) {
		int idx = OUTPUTS[i].expander;
		int pin = EXPANDER_PINS[OUTPUTS[i].port];
		_expanders[idx].pinMode(pin, OUTPUT);
	}
}

void
loop() {
	if (_input_change == 0) { return; }
	_input_change = 0; /* sync here */

	static u16 pin_read[array_len_(_expanders)];
	for (int i = 0; i < array_len_(_expanders); i += 1) {
		int irq_pin = _expanders[i].getLastInterruptPin();
		if (irq_pin != MCP23XXX_INT_ERR) {
			/* implicitly clears interrupt */
			pin_read[i] = _expanders[i].getCapturedInterrupt();
		}
	}

	/* Set output high if corresponding input is low */
	u16 pin_write[array_len_(_expanders)];
	memset(pin_write, 0, sizeof(pin_write));
	for (int i = 0; i < array_len_(INPUTS); i += 1) {
		u16 mask = 1 << INPUTS[i].port;
		if ((pin_read[INPUTS[i].expander] & mask) == 0) {
			mask = 1 << OUTPUTS[i].port;
			pin_write[OUTPUTS[i].expander] |= mask;
		}
	}

	for (int i = 0; i < array_len_(_expanders); i += 1) {
		_expanders[i].writeGPIOAB(pin_write[i]);
	}
}
