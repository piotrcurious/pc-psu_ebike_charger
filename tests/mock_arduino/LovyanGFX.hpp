#ifndef MOCK_LOVYANGFX_H
#define MOCK_LOVYANGFX_H

#include <string>
#include <cstdio>
#include <cstdint>

#define TFT_BLACK       0x0000
#define TFT_WHITE       0xFFFF
#define TFT_RED         0xF800
#define TFT_GREEN       0x07E0
#define TFT_BLUE        0x001F
#define TFT_YELLOW      0xFFE0
#define TFT_ORANGE      0xFD20
#define TFT_CYAN        0x07FF
#define TFT_MAGENTA     0xF81F
#define TFT_LIGHTGRAY   0xC618
#define TFT_DARKGREY    0x7BEF

namespace lgfx {
    struct Config_Bus {
        int spi_host;
        int spi_mode;
        int freq_write;
        int freq_read;
        bool spi_3wire;
        bool use_lock;
        int dma_channel;
        int pin_sclk;
        int pin_mosi;
        int pin_miso;
        int pin_dc;
    };

    struct Config_Panel {
        int pin_cs;
        int pin_rst;
        int pin_busy;
        int memory_width;
        int memory_height;
        int panel_width;
        int panel_height;
        int offset_x;
        int offset_y;
        int offset_rotation;
        int dummy_read_pixel;
        int dummy_read_bits;
        bool readable;
        bool invert;
        bool rgb_order;
        bool dlen_16bit;
        bool bus_shared;
    };

    class Bus_SPI {
    public:
        Config_Bus config() { return Config_Bus(); }
        void config(const Config_Bus& cfg) {}
    };

    class Panel_ILI9341 {
    public:
        Config_Panel config() { return Config_Panel(); }
        void config(const Config_Panel& cfg) {}
        void setBus(Bus_SPI* bus) {}
    };

    class LGFX_Device {
    public:
        void init() {}
        void startWrite() {}
        void endWrite() {}
        void fillScreen(uint32_t c) {}
        void fillRect(int x, int y, int w, int h, uint32_t c) {}
        void drawRect(int x, int y, int w, int h, uint32_t c) {}
        void drawLine(int x1, int y1, int x2, int y2, uint32_t c) {}
        void drawFastHLine(int x, int y, int w, uint32_t c) {}
        void setRotation(int r) {}
        void setCursor(int x, int y) {}
        void setTextColor(uint32_t c) {}
        void setTextSize(int s) {}
        void print(const char* s) {}
        void println(const char* s) {}
        template<typename... Args>
        void printf(const char* fmt, Args... args) {}
        void setPanel(Panel_ILI9341* panel) {}
    };
}

#define SPI_DMA_CH_AUTO 0
#define VSPI_HOST 1

#endif
