#ifndef AK_HOST_IDF_SPI_MASTER_H
#define AK_HOST_IDF_SPI_MASTER_H

#include <stddef.h>
#include <stdint.h>

#include "driver/rmt_types.h" /* esp_err_t, gpio_num_t */
/* IDF's SPI header takes FreeRTOS tick types in its queue calls, so this
 * stand-in does too - one definition of a tick for the whole stub tree. */
#include "freertos/FreeRTOS.h"

/*
 * The SPI master surface `src/arch/esp32/spi.c` calls, declared the way IDF
 * declares it.
 *
 * The one field worth reading twice is `spi_transaction_t::length`: it is in
 * **bits**, not bytes. It is the same shape of trap as `rmt_transmit()`'s byte
 * count - a unit that a plausible-looking call gets wrong, on a peripheral
 * this machine cannot run - which is why the model this compiles against
 * decodes the frame it is handed rather than trusting the caller's arithmetic.
 */

typedef int spi_host_device_t;
typedef int spi_dma_chan_t;
typedef struct host_spi_device *spi_device_handle_t;

#define SPI2_HOST 1
#define SPI_DMA_CH_AUTO 3

#define SPI_TRANS_CS_KEEP_ACTIVE (1 << 8)

typedef struct {
    int mosi_io_num;
    int miso_io_num;
    int sclk_io_num;
    int quadwp_io_num;
    int quadhd_io_num;
    int max_transfer_sz;
    struct {
        unsigned int isr_cpu_id : 2;
        unsigned int intr_flags : 6;
    } flags;
} spi_bus_config_t;

typedef struct {
    int clock_speed_hz;
    int mode;
    int spics_io_num;
    int queue_size;
    int command_bits;
    int address_bits;
    int dummy_bits;
} spi_device_interface_config_t;

typedef struct {
    size_t length;      /* in BITS, as IDF says */
    size_t rxlength;    /* in BITS; 0 means "the same as length" */
    void *user;
    const void *tx_buffer;
    void *rx_buffer;
    uint32_t flags;
} spi_transaction_t;

esp_err_t spi_bus_initialize(spi_host_device_t host_id,
                             const spi_bus_config_t *bus_config,
                             spi_dma_chan_t dma_chan);
esp_err_t spi_bus_add_device(spi_host_device_t host_id,
                             const spi_device_interface_config_t *dev_config,
                             spi_device_handle_t *handle);
esp_err_t spi_device_queue_trans(spi_device_handle_t handle,
                                 spi_transaction_t *trans_desc,
                                 TickType_t ticks_to_wait);
esp_err_t spi_device_get_trans_result(spi_device_handle_t handle,
                                      spi_transaction_t **trans_desc,
                                      TickType_t ticks_to_wait);

#endif /* AK_HOST_IDF_SPI_MASTER_H */
