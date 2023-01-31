void hspi_init();
void IRAM_ATTR hspi_trans(uint8_t cmd_data, uint8_t dout_bits, uint8_t din_bits);