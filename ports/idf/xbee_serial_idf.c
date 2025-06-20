#include "driver/uart.h"
#include "driver/gpio.h"
#include "xbee/serial.h"
#include "platform_config.h"
#include "esp_log.h"

const char* TAG = "xbee_serial_idf";
QueueHandle_t uart_queue;

bool_t xbee_ser_invalid(xbee_serial_t* serial) {
	return (serial == NULL || serial->port >= UART_NUM_MAX) ? 1 : 0;
}

int xbee_ser_write(xbee_serial_t* serial, const void FAR* buffer, int length) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	int result = uart_write_bytes(serial->port, buffer, length);

	if (result < 0) {
		return -EIO;
	}

	return (result < 0) ? -EIO : result;
}

int xbee_ser_read(xbee_serial_t* serial, void FAR* buffer, int bufsize) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	int result = uart_read_bytes(serial->port, buffer, bufsize, 0);

	if (result < 0) {
		return -EIO;
	}

	return (result < 0) ? -EIO : result;
}

int xbee_ser_open(xbee_serial_t* serial, uint32_t baudrate) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	uart_config_t uart_config = {
		.baud_rate = (int)baudrate,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = serial->hw_flow_control ? UART_HW_FLOWCTRL_CTS_RTS : UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 122, // Default threshold for RTS
	};

	ESP_ERROR_CHECK(uart_param_config(serial->port, &uart_config));

	ESP_ERROR_CHECK(uart_set_pin(serial->port, serial->tx_pin, serial->rx_pin,
		serial->rts_pin, serial->cts_pin));

	ESP_ERROR_CHECK(uart_driver_install(serial->port, 512, 512, 10, &uart_queue, 0));

	ESP_LOGD(TAG, "Serial port %d opened successfully, pins: tx: %d, rx: %d, rts: %d, cts: %d", serial->port,
		serial->tx_pin, serial->rx_pin, serial->rts_pin, serial->cts_pin);

	return 0;
}

int xbee_ser_baudrate(xbee_serial_t* serial, uint32_t baudrate) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	return 0;
}

int xbee_ser_flowcontrol(xbee_serial_t* serial, bool_t enabled) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	return 0;
}

int xbee_ser_set_rts(xbee_serial_t* serial, bool_t asserted) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	return 0;
}

int xbee_ser_get_cts(xbee_serial_t* serial) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	return 1;
}

int xbee_ser_tx_free(xbee_serial_t* serial) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	size_t free_size;
	uart_get_tx_buffer_free_size(serial->port, &free_size);

	return (int)free_size;
}

int xbee_ser_rx_used(xbee_serial_t* serial) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	size_t size;
	uart_get_buffered_data_len(serial->port, &size);

	return (int)size;
}

int xbee_ser_tx_flush(xbee_serial_t* serial) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	return (uart_flush(serial->port) == ESP_OK) ? 0 : -EIO;
}

int xbee_ser_rx_flush(xbee_serial_t* serial) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	return (uart_flush_input(serial->port) == ESP_OK) ? 0 : -EIO;
}

int xbee_ser_break(xbee_serial_t* serial, bool_t enabled) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	return 0;
}

const char* xbee_ser_portname(xbee_serial_t* serial) {
	if (xbee_ser_invalid(serial)) return NULL;

	static char port_name[16];
	snprintf(port_name, sizeof(port_name), "UART%d", serial->port);

	return port_name;
}

int xbee_ser_tx_used(xbee_serial_t* serial) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	size_t size;
	ESP_ERROR_CHECK(uart_get_buffered_data_len(serial->port, &size));

	return (int)size;
}

int xbee_ser_rx_free(xbee_serial_t* serial) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	size_t free_size;
	ESP_ERROR_CHECK(uart_get_buffered_data_len(serial->port, &free_size));

	return (int)free_size;
}

int xbee_ser_close(xbee_serial_t* serial) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	ESP_ERROR_CHECK(uart_driver_delete(serial->port));

	return 0;
}

int xbee_ser_putchar(xbee_serial_t* serial, uint8_t ch) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	int result = uart_write_bytes(serial->port, &ch, 1);

	if (result < 0) {
		return -EIO;
	}

	return (result < 0) ? -EIO : result;
}

int xbee_ser_getchar(xbee_serial_t* serial) {
	if (xbee_ser_invalid(serial)) return -EINVAL;

	uint8_t ch;
	int result = uart_read_bytes(serial->port, &ch, 1, 0);

	if (result < 0) {
		return -EIO;
	} else if (result == 0) {
		return -ENODATA; // No data available
	}

	return ch;
}