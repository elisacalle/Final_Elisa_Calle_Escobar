//Final Elisa Calle Escobar

//PUNTO#3
#define MCP_SPI_HOST  SPI2_HOST
#define MCP_SPI_FREQ  1000000   // 1 MHz
#define MCP_SPI_MODE  0        

#define MCP_CMD_WRITE 0x00      // C1:C0 = 00
#define MCP_CMD_READ  0x03      // C1:C0 = 11

static spi_device_handle_t mcp4132_handle;

static uint8_t mcp4132_build_command(uint8_t address, uint8_t command, uint16_t value)
{
    uint8_t cmd = 0; 

    cmd |= (address & 0x0F) << 4;  // AD3:AD0
    cmd |= (command & 0x03) << 2;  // C1:C0
    cmd |= (value >> 8) & 0x03;    // D9:D8

    return cmd;
}

// a) Inicializar bus SPI y dispositivo MCP4132

esp_err_t spi_bus_init(void)
{
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    esp_err_t ret = spi_bus_initialize(
        MCP_SPI_HOST,
        &bus_cfg,
        SPI_DMA_CH_AUTO
    );

    if (ret != ESP_OK) {
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = MCP_SPI_FREQ,
        .mode = MCP_SPI_MODE,
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };

    return spi_bus_add_device(
        MCP_SPI_HOST,
        &dev_cfg,
        &mcp4132_handle
    );
}

// b) Escribir un registro del MCP4132

esp_err_t mcp4132_write_register(uint8_t address, uint16_t value)
{
    uint8_t tx_data[2];

    tx_data[0] = mcp4132_build_command(address, MCP_CMD_WRITE, value);
    tx_data[1] = value & 0xFF;

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));

    transaction.length = 16;
    transaction.tx_buffer = tx_data;

    return spi_device_transmit(mcp4132_handle, &transaction);
}

// c) Leer un registro del MCP4132

uint16_t mcp4132_read_register(uint8_t address)
{
    uint8_t tx_data[2];
    uint8_t rx_data[2];

    tx_data[0] = mcp4132_build_command(address, MCP_CMD_READ, 0);
    tx_data[1] = 0x00;  // dummy byte para generar reloj SPI

    memset(rx_data, 0, sizeof(rx_data));

    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));

    transaction.length = 16;
    transaction.tx_buffer = tx_data;
    transaction.rx_buffer = rx_data;

    esp_err_t ret = spi_device_transmit(mcp4132_handle, &transaction);

    if (ret != ESP_OK) {
        return 0xFFFF; 
    }

    uint16_t value = ((rx_data[0] & 0x01) << 8) | rx_data[1];

    return value;
}




//PUNTO 4
#define MCP4132_REG_WIPER0  0x00

#define MCP4132_N_MIN       0
#define MCP4132_N_MAX       128

#define RAB_OHMS            10000.0f
#define RW_OHMS             75.0f
#define C_FARADS            10e-9f
#define PI_VALUE            3.14159265f

// Función anterior
extern esp_err_t mcp4132_write_register(uint8_t address, uint16_t value);

// a) Escribir directamente N en el wiper 0

esp_err_t mcp4132_set_wiper(uint8_t n)
{
    if (n > MCP4132_N_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    return mcp4132_write_register(MCP4132_REG_WIPER0, (uint16_t)n);
}

// b) Configurar frecuencia de corte

esp_err_t mcp4132_set_cutoff_frequency(float fc_hz)
{
    if (fc_hz <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    // Ecuación 1 despejada:
    // RWB = 1 / (2*pi*fc*C)
    float target_rwb = 1.0f / (2.0f * PI_VALUE * fc_hz * C_FARADS);

    // Ecuación 2 despejada:
    // RWB = ((RAB*N)/128) + RW
    // N = ((RWB - RW) * 128) / RAB
    float n_float = ((target_rwb - RW_OHMS) * 128.0f) / RAB_OHMS;

    // N más cercano
    int n = (int)(n_float + 0.5f);

    // Rango permitido
    if (n < MCP4132_N_MIN) {
        n = MCP4132_N_MIN;
    }

    if (n > MCP4132_N_MAX) {
        n = MCP4132_N_MAX;
    }

    return mcp4132_set_wiper((uint8_t)n);
}


//PUNTO 5
#define ADC_CHANNEL     ADC_CHANNEL_6
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define ADC_BITWIDTH    ADC_BITWIDTH_12

//Suposición pines
#define UART_PORT       UART_NUM_1
#define UART_TX_PIN     GPIO_NUM_17
#define UART_RX_PIN     GPIO_NUM_16
#define UART_BAUDRATE   115200
#define UART_BUF_SIZE   256

// Umbrales
#define ADC_MAX_VALUE   4095.0f
#define VREF            3.3f

#define THRESHOLD_HIGH  1.4f
#define THRESHOLD_LOW   0.9f

#define WIPER_HIGH      95
#define WIPER_LOW       42

static adc_oneshot_unit_handle_t adc_handle;
static volatile bool sample_flag = false;
static uint8_t current_wiper = 0;

// Funciones punto anterior
extern esp_err_t spi_bus_init(void);
extern esp_err_t mcp4132_set_wiper(uint8_t n);

// Timer callback 1 kHz
static void sample_timer_callback(void *arg)
{
    sample_flag = true;
}

void app_main(void)
{
    spi_bus_init();

    //ADC 12 bits
    adc_oneshot_unit_init_cfg_t adc_init_cfg = {
        .unit_id = ADC_UNIT_1,
    };

    adc_oneshot_new_unit(&adc_init_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t adc_chan_cfg = {
        .bitwidth = ADC_BITWIDTH,
        .atten = ADC_ATTEN,
    };

    adc_oneshot_config_channel(
        adc_handle,
        ADC_CHANNEL,
        &adc_chan_cfg
    );

    // Configurar UART

    uart_config_t uart_cfg = {
        .baud_rate = UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_PORT, &uart_cfg);

    uart_set_pin(
        UART_PORT,
        UART_TX_PIN,
        UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );

    uart_driver_install(
        UART_PORT,
        UART_BUF_SIZE,
        UART_BUF_SIZE,
        0,
        NULL,
        0
    );

    const esp_timer_create_args_t timer_args = {
        .callback = sample_timer_callback,
        .name = "adc_sample_timer"
    };

    esp_timer_handle_t sample_timer;

    esp_timer_create(&timer_args, &sample_timer);
    esp_timer_start_periodic(sample_timer, 1000);

    uart_write_bytes(
        UART_PORT,
        "Sistema iniciado. Muestreo ADC a 1 kHz\r\n",
        strlen("Sistema iniciado. Muestreo ADC a 1 kHz\r\n")
    );

    while (1) {
        if (sample_flag) {
            sample_flag = false;

            int raw = 0;
            adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);

            float voltage = ((float)raw * VREF) / ADC_MAX_VALUE;

            if (voltage > THRESHOLD_HIGH && current_wiper != WIPER_HIGH) {
                current_wiper = WIPER_HIGH;
                mcp4132_set_wiper(current_wiper);

                char msg[100];
                snprintf(
                    msg,
                    sizeof(msg),
                    "Voltaje %.2f V > 1.4 V. Wiper actualizado: N=%d\r\n",
                    voltage,
                    current_wiper
                );

                uart_write_bytes(UART_PORT, msg, strlen(msg));
            }

            else if (voltage < THRESHOLD_LOW && current_wiper != WIPER_LOW) {
                current_wiper = WIPER_LOW;
                mcp4132_set_wiper(current_wiper);

                char msg[100];
                snprintf(
                    msg,
                    sizeof(msg),
                    "Voltaje %.2f V < 0.9 V. Wiper actualizado: N=%d\r\n",
                    voltage,
                    current_wiper
                );

                uart_write_bytes(UART_PORT, msg, strlen(msg));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}