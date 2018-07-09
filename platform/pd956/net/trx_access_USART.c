

#include "usart_spi.h"
#include "contiki.h"
#include "trx_access.h"
#include "rtimer-arch.h"

static irq_handler_t irq_hdl_trx = NULL;

#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    static rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)


struct usart_spi_device SPI_AT86RFX_DEVICE = {
	.id = AT86RFX_SPI_CS
};



AT86RFX_ISR()
{
	/*Clearing the interrupt*/
	trx_irq_flag_clr();
  	/*Calling the interrupt routines*/
	if (irq_hdl_trx) {
		irq_hdl_trx();
	}
}

void trx_spi_init(void)
{
	/* Initialize SPI in master mode to access the transceiver */

	usart_spi_init(AT86RFX_SPI);
	usart_spi_setup_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE, SPI_MODE_0,
			AT86RFX_SPI_BAUDRATE, 0);
	usart_spi_enable(AT86RFX_SPI);
	AT86RFX_INTC_INIT();

}

void PhyReset(void)
{
	/* Ensure control lines have correct levels. */
	RST_HIGH();
	SLP_TR_LOW();

	/* Wait typical time of timer TR1. */
	//delay_us(330);
	BUSYWAIT_UNTIL(0, 1 * RTIMER_SECOND / 1000);

	RST_LOW();
	BUSYWAIT_UNTIL(0, 1 * RTIMER_SECOND / 1000);
	RST_HIGH();
}

uint8_t trx_reg_read(uint8_t addr)
{

	uint8_t register_value = 0;
	uint8_t addr_val = addr | READ_ACCESS_COMMAND;

	ENTER_TRX_CRITICAL_REGION();

	usart_spi_select_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);
	usart_spi_write_packet(AT86RFX_SPI, &addr_val, 1);
	usart_spi_read_packet(AT86RFX_SPI, &register_value, 1);
	usart_spi_deselect_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);

	LEAVE_TRX_CRITICAL_REGION();

	return register_value;
}

void trx_reg_write(uint8_t addr, uint8_t data)
{
	ENTER_TRX_CRITICAL_REGION();

	addr |= WRITE_ACCESS_COMMAND;

	usart_spi_select_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);
	usart_spi_write_packet(AT86RFX_SPI, &addr, 1);
	usart_spi_write_packet(AT86RFX_SPI, &data, 1);
	usart_spi_deselect_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);

	LEAVE_TRX_CRITICAL_REGION();
}

void trx_irq_init(FUNC_PTR trx_irq_cb)
{
	/*
	 * Set the handler function.
	 * The handler is set before enabling the interrupt to prepare for
	 * spurious
	 * interrupts, that can pop up the moment they are enabled
	 */
	irq_hdl_trx = (irq_handler_t)trx_irq_cb;
}

uint8_t trx_bit_read(uint8_t addr, uint8_t mask, uint8_t pos)
{
	uint8_t ret;
	ret = trx_reg_read(addr);
	ret &= mask;
	ret >>= pos;
	return ret;
}

void trx_bit_write(uint8_t reg_addr, uint8_t mask, uint8_t pos,
		uint8_t new_value)
{
	uint8_t current_reg_value;
	current_reg_value = trx_reg_read(reg_addr);
	current_reg_value &= ~mask;
	new_value <<= pos;
	new_value &= mask;
	new_value |= current_reg_value;
	trx_reg_write(reg_addr, new_value);
}

void trx_frame_read(uint8_t *data, uint8_t length)
{
	/*Saving the current interrupt status & disabling the global interrupt
	**/
	ENTER_TRX_CRITICAL_REGION();


	uint8_t temp;
	/* Start SPI transaction by pulling SEL low */
	usart_spi_select_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);

	temp = TRX_CMD_FR;

	/* Send the command byte */
	usart_spi_write_packet(AT86RFX_SPI, &temp, 1);

	usart_spi_read_packet(AT86RFX_SPI, data, length);

	/* Stop the SPI transaction by setting SEL high */
	usart_spi_deselect_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);


	/*Restoring the interrupt status which was stored & enabling the global
	 * interrupt */
	LEAVE_TRX_CRITICAL_REGION();
}



void trx_frame_write(uint8_t *data, uint8_t length)
{
	uint8_t temp;

	/*Saving the current interrupt status & disabling the global interrupt
	**/
	ENTER_TRX_CRITICAL_REGION();


	/* Start SPI transaction by pulling SEL low */
	usart_spi_select_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);

	temp = TRX_CMD_FW;

	/* Send the command byte */
	usart_spi_write_packet(AT86RFX_SPI, &temp, 1);
	usart_spi_write_packet(AT86RFX_SPI, data, length);

	/* Stop the SPI transaction by setting SEL high */
	usart_spi_deselect_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);


	/*Restoring the interrupt status which was stored & enabling the global
	 * interrupt */
	LEAVE_TRX_CRITICAL_REGION();
}

/**
 * @brief Writes data into SRAM of the transceiver
 *
 * This function writes data into the SRAM of the transceiver
 *
 * @param addr Start address in the SRAM for the write operation
 * @param data Pointer to the data to be written into SRAM
 * @param length Number of bytes to be written into SRAM
 */
void trx_sram_write(uint8_t addr, uint8_t *data, uint8_t length)
{
	uint8_t temp;

	/*Saving the current interrupt status & disabling the global interrupt
	**/
	ENTER_TRX_CRITICAL_REGION();

	/* Start SPI transaction by pulling SEL low */
	usart_spi_select_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);

	/* Send the command byte */
	temp = TRX_CMD_SW;

	/* Send the command byte */
	usart_spi_write_packet(AT86RFX_SPI, &temp, 1);
	while (!usart_spi_is_tx_empty(AT86RFX_SPI)) {
	}

	/* Send the address from which the write operation should start */
	usart_spi_write_packet(AT86RFX_SPI, &addr, 1);
	while (!usart_spi_is_tx_empty(AT86RFX_SPI)) {
	}

	usart_spi_write_packet(AT86RFX_SPI, data, length);

	/* Stop the SPI transaction by setting SEL high */
	usart_spi_deselect_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);


	/*Restoring the interrupt status which was stored & enabling the global
	 * interrupt */
	LEAVE_TRX_CRITICAL_REGION();
}

/**
 * @brief Reads data from SRAM of the transceiver
 *
 * This function reads from the SRAM of the transceiver
 *
 * @param[in] addr Start address in SRAM for read operation
 * @param[out] data Pointer to the location where data stored
 * @param[in] length Number of bytes to be read from SRAM
 */
void trx_sram_read(uint8_t addr, uint8_t *data, uint8_t length)
{
	//delay_us(1); /* wap_rf4ce */
	//BUSYWAIT_UNTIL(0, 1 * RTIMER_SECOND / 1000);

	/*Saving the current interrupt status & disabling the global interrupt
	**/
	ENTER_TRX_CRITICAL_REGION();

	uint8_t temp;
	/* Start SPI transaction by pulling SEL low */
	usart_spi_select_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);

	temp = TRX_CMD_SR;

	/* Send the command byte */
	usart_spi_write_packet(AT86RFX_SPI, &temp, 1);
	while (!usart_spi_is_tx_empty(AT86RFX_SPI)) {
	}

	/* Send the address from which the read operation should start */
	/* Upload the received byte in the user provided location */
	usart_spi_write_packet(AT86RFX_SPI, &addr, 1);
	while (!usart_spi_is_tx_empty(AT86RFX_SPI)) {
	}

	usart_spi_read_packet(AT86RFX_SPI, data, length);

	/* Stop the SPI transaction by setting SEL high */
	usart_spi_deselect_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);


	/*Restoring the interrupt status which was stored & enabling the global
	 * interrupt */
	LEAVE_TRX_CRITICAL_REGION();
}


/**
 * @brief Writes and reads data into/from SRAM of the transceiver
 *
 * This function writes data into the SRAM of the transceiver and
 * simultaneously reads the bytes.
 *
 * @param addr Start address in the SRAM for the write operation
 * @param idata Pointer to the data written/read into/from SRAM
 * @param length Number of bytes written/read into/from SRAM
 */
void trx_aes_wrrd(uint8_t addr, uint8_t *idata, uint8_t length)
{
	uint8_t *odata;
	uint8_t temp;

	//delay_us(1); /* wap_rf4ce */
	BUSYWAIT_UNTIL(0, 1 * RTIMER_SECOND / 1000);

	ENTER_TRX_REGION();

#ifdef NON_BLOCKING_SPI
	while (spi_state != SPI_IDLE) {
		/* wait until SPI gets available */
	}
#endif

	/* Start SPI transaction by pulling SEL low */
	usart_spi_select_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);

	/* Send the command byte */
	temp = TRX_CMD_SW;
	usart_spi_write_packet(AT86RFX_SPI, &temp, 1);
	while (!usart_spi_is_tx_empty(AT86RFX_SPI)) {
	}

	/* write SRAM start address */
	usart_spi_write_packet(AT86RFX_SPI, &addr, 1);
	while (!usart_spi_is_tx_empty(AT86RFX_SPI)) {
	}

	/* now transfer data */
	odata = idata;

	/* write data byte 0 - the obtained value in SPDR is meaningless */
	usart_spi_write_packet(AT86RFX_SPI, idata++, 1);

	/* Reading Spi Data for the length specified */
	while (length > 0) {
		usart_spi_write_packet(AT86RFX_SPI, idata++, 1);
		while (!usart_spi_is_tx_empty(AT86RFX_SPI)) {
		}
		usart_spi_read_single(AT86RFX_SPI, odata++);
		length--;
	}

	/* To get the last data byte, write some dummy byte */
	usart_spi_read_packet(AT86RFX_SPI, odata, 1);

	/* Stop the SPI transaction by setting SEL high */
	usart_spi_deselect_device(AT86RFX_SPI, &SPI_AT86RFX_DEVICE);

	LEAVE_TRX_REGION();
}
