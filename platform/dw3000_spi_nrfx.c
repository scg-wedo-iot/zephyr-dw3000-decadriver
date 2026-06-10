/*
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 * Copyright 2021 (c) Callender-Consulting LLC.
 */

#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>

#include <soc.h>
#include <nrfx_spim.h>

#include "dw3000_spi.h"

#include "version.h"

/* This file implements the SPI functions required by decadriver */

LOG_MODULE_DECLARE(dw3000, CONFIG_DW3000_LOG_LEVEL);

#define DW_INST DT_INST(0, decawave_dw3000)
#define DW_SPI	DT_PARENT(DT_INST(0, decawave_dw3000))

#define SPI_BUS_ADDR DT_REG_ADDR(DW_SPI)

#if (SPI_BUS_ADDR == NRF_SPIM0_BASE)
	#define SPI_INSTANCE_NUM 0
#elif (SPI_BUS_ADDR == NRF_SPIM1_BASE)
	#define SPI_INSTANCE_NUM 1
#elif (SPI_BUS_ADDR == NRF_SPIM2_BASE)
	#define SPI_INSTANCE_NUM 2
#elif (SPI_BUS_ADDR == NRF_SPIM3_BASE)
	#define SPI_INSTANCE_NUM 3
#else
	#error "Unsupported SPIM instance address"
#endif

#define SPIM_INST_IDX SPI_INSTANCE_NUM
#define SPIM_INST NRFX_CONCAT_2(NRF_SPIM, SPIM_INST_IDX)

#define SPIM_NODE DT_NODELABEL(NRFX_CONCAT_2(spi, SPIM_INST_IDX))
PINCTRL_DT_DEFINE(SPIM_NODE);

#define DW3000_SPI_BUFFER_SIZE 255 // EasyDMA buffer size limit to 255 bytes

#if KERNEL_VERSION_MAJOR > 4                                                   \
	|| (KERNEL_VERSION_MAJOR == 4 && KERNEL_VERSION_MINOR >= 3)
/* Zephyr >= 4.3: explicit delay argument is deprecated; the CS delay comes
 * from the spi-cs-{setup,hold}-delay-ns devicetree properties (default 0). */
static struct spi_cs_control cs_ctrl = SPI_CS_CONTROL_INIT(DW_INST);
#elif KERNEL_VERSION_MAJOR > 3                                                 \
	|| (KERNEL_VERSION_MAJOR == 3 && KERNEL_VERSION_MINOR >= 4)
static struct spi_cs_control cs_ctrl = SPI_CS_CONTROL_INIT(DW_INST, 0);
#else
static struct spi_cs_control* cs_ctrl = SPI_CS_CONTROL_PTR_DT(DW_INST, 0);
#endif

/* nrfx 4.0: NRFX_SPIM_INSTANCE expects the peripheral base address (e.g.
 * NRF_SPIM3) rather than the instance index. SPIM_INST resolves to NRF_SPIM<n>. */
static nrfx_spim_t spim = NRFX_SPIM_INSTANCE(SPIM_INST);
static uint32_t max_spi_frequency = DT_PROP(DW_INST, spi_max_frequency);
static bool spim_initialized = false;
static volatile bool transfer_finished = false;

__aligned(4) static uint8_t g_spi_tx_buf[DW3000_SPI_BUFFER_SIZE] = {0};
__aligned(4) static uint8_t g_spi_rx_buf[DW3000_SPI_BUFFER_SIZE] = {0};

static void spim_handler(const nrfx_spim_event_t *p_event, void *p_context)
{
	if (p_event->type == NRFX_SPIM_EVENT_DONE) {
		transfer_finished = true;
	}
}

/* nrfx 4.0: NRFX_SPIM_INST_HANDLER_GET was removed and the generic
 * nrfx_spim_irq_handler() now takes the driver instance pointer. Wrap it in a
 * direct ISR and register it with IRQ_DIRECT_CONNECT so the SPIM IRQ stays
 * zero-latency (low-latency UWB SPI) and does not take a software ISR-table
 * slot - which would clash with the (unused) Zephyr SPI device that also binds
 * this controller node. Completion is signalled via the transfer_finished
 * flag, so the ISR never needs to reschedule. */
ISR_DIRECT_DECLARE(dw3000_spim_isr)
{
	nrfx_spim_irq_handler(&spim);
	/* Self-guarded by CONFIG_PM (no-op when system PM is disabled). Completion
	 * is polled via transfer_finished, so no reschedule is requested. */
	ISR_DIRECT_PM();
	return 0;
}

int dw3000_spi_init(void)
{
	IRQ_DIRECT_CONNECT(DT_IRQN(SPIM_NODE), DT_IRQ(SPIM_NODE, priority),
					   dw3000_spim_isr,
					   IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? IRQ_ZERO_LATENCY : 0);
	irq_enable(DT_IRQN(SPIM_NODE));

	dw3000_spi_speed_slow();

	// initialized correctly at boot but after fini we need to reconfigure
	gpio_pin_configure_dt(&cs_ctrl.gpio, GPIO_OUTPUT_HIGH);

	return 0;
}

void dw3000_spi_speed_slow(void)
{
	int err;

	nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(
		NRF_SPIM_PIN_NOT_CONNECTED,
		NRF_SPIM_PIN_NOT_CONNECTED,
		NRF_SPIM_PIN_NOT_CONNECTED,
		NRF_SPIM_PIN_NOT_CONNECTED
	);
	spim_config.frequency = NRFX_MHZ_TO_HZ(2);
	spim_config.skip_gpio_cfg = true;
	spim_config.skip_psel_cfg = true;

	if (spim_initialized) {
		err = nrfx_spim_reconfigure(&spim, &spim_config);
		if (err < 0) {
			LOG_ERR("nrfx_spim_reconfigure() slow speed failed: 0x%08x", err);
		}
		return;
	}

	err = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(SPIM_NODE),
							  PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("pinctrl_apply_state() slow speed failed: 0x%08x", err);
		return;
	}

	err = nrfx_spim_init(&spim, &spim_config, spim_handler, NULL);
	if (err < 0) {
		LOG_ERR("nrfx_spim_init() slow speed failed: 0x%08x", err);
		return;
	}

	spim_initialized = true;
}

void dw3000_spi_speed_fast(void)
{
	int err;

	nrfx_spim_config_t spim_config = NRFX_SPIM_DEFAULT_CONFIG(
		NRF_SPIM_PIN_NOT_CONNECTED,
		NRF_SPIM_PIN_NOT_CONNECTED,
		NRF_SPIM_PIN_NOT_CONNECTED,
		NRF_SPIM_PIN_NOT_CONNECTED
	);
	spim_config.frequency = max_spi_frequency;
	spim_config.skip_gpio_cfg = true;
	spim_config.skip_psel_cfg = true;

	if (spim_initialized) {
		err = nrfx_spim_reconfigure(&spim, &spim_config);
		if (err < 0) {
			LOG_ERR("nrfx_spim_reconfigure() high speed failed: 0x%08x", err);
		}
		return;
	}

	err = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(SPIM_NODE),
							  PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		LOG_ERR("pinctrl_apply_state() fast speed failed: 0x%08x", err);
		return;
	}

	err = nrfx_spim_init(&spim, &spim_config, spim_handler, NULL);
	if (err < 0) {
		LOG_ERR("nrfx_spim_init() fast speed failed: 0x%08x", err);
		return;
	}

	spim_initialized = true;
}

void dw3000_spi_fini(void)
{
	if (spim_initialized) {
		nrfx_spim_uninit(&spim);
		spim_initialized = false;
	}

	gpio_pin_configure_dt(&cs_ctrl.gpio, GPIO_DISCONNECTED);
}

int32_t dw3000_spi_write_crc(uint16_t headerLength, const uint8_t* headerBuffer,
							 uint16_t bodyLength, const uint8_t* bodyBuffer,
							 uint8_t crc8)
{
	uint8_t *p_buf;
	uint32_t total_length = headerLength + bodyLength + sizeof(crc8);

	p_buf = g_spi_tx_buf;
	memcpy(p_buf, headerBuffer, headerLength);
	p_buf += headerLength;
	memcpy(p_buf, bodyBuffer, bodyLength);
	p_buf += bodyLength;
	memcpy(p_buf, &crc8, 1);

	int err;
	nrfx_spim_xfer_desc_t xfer_desc = {
		.p_tx_buffer = g_spi_tx_buf,
		.tx_length = total_length,
		.p_rx_buffer = g_spi_rx_buf,
		.rx_length = total_length,
	};

	transfer_finished = false;
	gpio_pin_set_dt(&cs_ctrl.gpio, 1);
	err = nrfx_spim_xfer(&spim, &xfer_desc, 0);
	if (err < 0) {
		LOG_ERR("nrfx_spim_xfer() failed: 0x%08x", err);
		gpio_pin_set_dt(&cs_ctrl.gpio, 0);
		return -EIO;
	}
	while (!transfer_finished);
	gpio_pin_set_dt(&cs_ctrl.gpio, 0);

	return 0;
}

int32_t dw3000_spi_write(uint16_t headerLength, const uint8_t* headerBuffer,
						 uint16_t bodyLength, const uint8_t* bodyBuffer)
{
	uint8_t *p_buf;
	uint32_t total_length = headerLength + bodyLength;

	p_buf = g_spi_tx_buf;
	memcpy(p_buf, headerBuffer, headerLength);
	p_buf += headerLength;
	memcpy(p_buf, bodyBuffer, bodyLength);

	int err;
	nrfx_spim_xfer_desc_t xfer_desc = {
		.p_tx_buffer = g_spi_tx_buf,
		.tx_length = total_length,
		.p_rx_buffer = g_spi_rx_buf,
		.rx_length = total_length,
	};

	transfer_finished = false;
	gpio_pin_set_dt(&cs_ctrl.gpio, 1);
	err = nrfx_spim_xfer(&spim, &xfer_desc, 0);
	if (err < 0) {
		LOG_ERR("nrfx_spim_xfer() failed: 0x%08x", err);
		gpio_pin_set_dt(&cs_ctrl.gpio, 0);
		return -EIO;
	}
	while (!transfer_finished);
	gpio_pin_set_dt(&cs_ctrl.gpio, 0);

	return 0;
}

int32_t dw3000_spi_read(uint16_t headerLength, uint8_t* headerBuffer,
						uint16_t readLength, uint8_t* readBuffer)
{
	uint8_t *p_buf;
	uint32_t total_length = headerLength + readLength;

	p_buf = g_spi_tx_buf;
	memcpy(p_buf, headerBuffer, headerLength);
	p_buf += headerLength;
	memset(p_buf, 0x00, readLength);

	int err;
	nrfx_spim_xfer_desc_t xfer_desc = {
		.p_tx_buffer = g_spi_tx_buf,
		.tx_length = total_length,
		.p_rx_buffer = g_spi_rx_buf,
		.rx_length = total_length,
	};

	transfer_finished = false;
	gpio_pin_set_dt(&cs_ctrl.gpio, 1);
	err = nrfx_spim_xfer(&spim, &xfer_desc, 0);
	if (err < 0) {
		LOG_ERR("nrfx_spim_xfer() failed: 0x%08x", err);
		gpio_pin_set_dt(&cs_ctrl.gpio, 0);
		return -EIO;
	}
	while (!transfer_finished);
	gpio_pin_set_dt(&cs_ctrl.gpio, 0);

	memcpy(readBuffer, g_spi_rx_buf + headerLength, readLength);

	return 0;
}

void dw3000_spi_wakeup()
{
	/* CS pin should be configured as active low
	 * To wake up, we set CS to 1, which will pull it low, for 500us */
#if KERNEL_VERSION_MAJOR > 3                                                   \
	|| (KERNEL_VERSION_MAJOR == 3 && KERNEL_VERSION_MINOR >= 4)
	gpio_pin_set_dt(&cs_ctrl.gpio, 1);
	k_sleep(K_USEC(500));
	gpio_pin_set_dt(&cs_ctrl.gpio, 0);
#else
	gpio_pin_set_dt(&cs_ctrl->gpio, 1);
	k_sleep(K_USEC(500));
	gpio_pin_set_dt(&cs_ctrl->gpio, 0);
#endif
}
