#pragma once
#include <tft_io.hpp>
#include <ili9341.hpp>
#include <sh200q.hpp>
namespace arduino {

class m5stack {
    constexpr static const uint8_t spi_host = VSPI;
    constexpr static const int8_t lcd_pin_bl = 32;
    constexpr static const int8_t lcd_pin_dc = 27;
    constexpr static const int8_t lcd_pin_cs = 14;
    constexpr static const int8_t spi_pin_mosi = 23;
    constexpr static const int8_t spi_pin_clk = 18;
    constexpr static const int8_t lcd_pin_rst = 33;
    constexpr static const int8_t spi_pin_miso = 19;

    using spi_type = tft_spi_ex<spi_host,lcd_pin_cs,spi_pin_mosi,spi_pin_miso,spi_pin_clk,SPI_MODE0,false,320*240*2+8,2>;
    using lcd_type = ili9342c<lcd_pin_dc,lcd_pin_rst,lcd_pin_bl,spi_type,1,true,270,50>;
    using spatial_type = sh200q;
public:
    lcd_type m_lcd;
    spatial_type m_spatial;
    using pixel_type = typename lcd_type::pixel_type;
    using caps = typename lcd_type::caps;
    inline m5stack() : m_spatial(Wire) {

    }
    inline lcd_type lcd() {
        initialize();
        return m_lcd;
    }
    inline spatial_type spatial() {
        initialize();
        return m_spatial;
    }
    inline bool initialized() const { return m_spatial.initialized(); }
    inline bool initialize() {
        if(!initialized()) {
            Wire.begin(21, 22);
            m_spatial.initialize();
        }
        return m_spatial.initialized();
    } 
    inline gfx::size16 dimensions() const {
        return m_lcd.dimensions();
    }
    inline gfx::rect16 bounds() const {
        return m_lcd.bounds();
    }
    inline gfx::gfx_result point(gfx::point16 location, pixel_type color) {
        return m_lcd.point(location,color);
    }
    inline gfx::gfx_result point_async(gfx::point16 location, pixel_type color) {
        return m_lcd.point_async(location, color);
    }
    inline gfx::gfx_result point(gfx::point16 location, pixel_type* out_color) const {
        return m_lcd.point(location,out_color);
    }
    inline gfx::gfx_result fill(const gfx::rect16& bounds, pixel_type color) {
        return m_lcd.fill(bounds,color);
    }
    inline gfx::gfx_result fill_async(const gfx::rect16& bounds, pixel_type color) {
        return m_lcd.fill_async(bounds,color);
    }
    inline gfx::gfx_result clear(const gfx::rect16& bounds) {
        return m_lcd.clear(bounds);
    }
    inline gfx::gfx_result clear_async(const gfx::rect16& bounds) {
        return m_lcd.clear_async(bounds);
    }
    template <typename Source>
    inline gfx::gfx_result copy_from(const gfx::rect16& src_rect, const Source& src, gfx::point16 location) {
        return m_lcd.copy_from(src_rect,src,location);
    }
    template <typename Source>
    inline gfx::gfx_result copy_from_async(const gfx::rect16& src_rect, const Source& src, gfx::point16 location) {
        return m_lcd.copy_from_async(src_rect,src,location);
    }
    template <typename Destination>
    inline gfx::gfx_result copy_to(const gfx::rect16& src_rect, Destination& dst, gfx::point16 location) const {
        return m_lcd.copy_to(src_rect,dst,location);
    }
    template <typename Destination>
    inline gfx::gfx_result copy_to_async(const gfx::rect16& src_rect, Destination& dst, gfx::point16 location) const {
        return m_lcd.copy_to_async(src_rect,dst,location);
    }

    inline gfx::gfx_result commit_batch() {
        return m_lcd.commit_batch();
    }
    inline gfx::gfx_result commit_batch_async() {
        return m_lcd.commit_batch_async();
    }
    inline gfx::gfx_result begin_batch(const gfx::rect16& bounds) {
        return m_lcd.begin_batch(bounds);
    }
    inline gfx::gfx_result begin_batch_async(const gfx::rect16& bounds) {
        return m_lcd.begin_batch_async(bounds);
    }
    inline gfx::gfx_result write_batch(pixel_type color) {
        return m_lcd.write_batch(color);
    }
    inline gfx::gfx_result write_batch_async(pixel_type color) {
        return m_lcd.write_batch_async(color);
    }
    inline gfx::gfx_result wait_all_async() {
        return m_lcd.wait_all_async();
    }
};
}