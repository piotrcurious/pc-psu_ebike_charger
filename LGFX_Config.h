#ifndef LGFX_CONFIG_H
#define LGFX_CONFIG_H

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "config_esp32.h"

class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9341 _panel_instance;
    lgfx::Bus_SPI       _bus_instance;

public:
    LGFX(void) {
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host = VSPI_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 40000000;
            cfg.freq_read  = 16000000;
            cfg.spi_3wire  = false;
            cfg.use_lock   = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = TFT_SCLK;
            cfg.pin_mosi = TFT_MOSI;
            cfg.pin_miso = -1;
            cfg.pin_dc   = TFT_DC;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs           =    TFT_CS;
            cfg.pin_rst          =    TFT_RST;
            cfg.pin_busy         =    -1;
            cfg.memory_width     =   240;
            cfg.memory_height    =   320;
            cfg.panel_width      =   240;
            cfg.panel_height     =   320;
            cfg.offset_x         =     0;
            cfg.offset_y         =     0;
            cfg.offset_rotation  =     1; // Landscape
            cfg.dummy_read_pixel =     8;
            cfg.dummy_read_bits  =     1;
            cfg.readable         =  true;
            cfg.invert           = false;
            cfg.rgb_order        = false;
            cfg.dlen_16bit       = false;
            cfg.bus_shared       =  true;
            _panel_instance.config(cfg);
        }
        setPanel(&_panel_instance);
    }
};

#endif
