#ifndef OLED_DISPLAY_HPP
#define OLED_DISPLAY_HPP

#include <cstdint>
#include <cstddef>
#include <I2C.hpp>


namespace Devices
{
  template<
      typename I2CinterfaceT
    , uint8_t I2Caddress
    , typename SdaPinT
    , typename SclPinT>
  class OledDisplay
  {
    public:
      void init()
      {
        // send commands for initialization
        // 1. Turn off the display
        status = SSD1306_WriteCommand(dev, SSD1306_DISPLAY_OFF);
        errNum += (status != HAL_OK);
        // 2. Set MUX Ratio to 64
        status = SSD1306_WriteCommand(dev, SSD1306_SET_MUX_RATIO);
        errNum += (status != HAL_OK);
        status = SSD1306_WriteCommand(dev, 0x3F);	// mux ratio is n + 1, write 63 for mux ratio 64
        errNum += (status != HAL_OK);
        // 3. Set page addressing mode
        status = SSD1306_WriteCommand(dev, SSD1306_SET_MEM_ADDR_MODE);
        errNum += (status != HAL_OK);
        status = SSD1306_WriteCommand(dev, SSD1306_PAGE_ADDR_MODE);
        errNum += (status != HAL_OK);
        // 4. Set the column start address to zero
        // first, the four less significant bits
        status = SSD1306_WriteCommand(dev, SSD1306_SET_LOWER_COL_ADDR | 0x00);
        errNum += (status != HAL_OK);
        // then, the four more significant bits
        status = SSD1306_WriteCommand(dev, SSD1306_SET_HIGHER_COL_ADDR | 0x00);
        errNum += (status != HAL_OK);
        // 5. Set GDDRAM page start address
        status = SSD1306_WriteCommand(dev, SSD1306_SET_PAGE_START_ADDR | 0x00);
        errNum += (status != HAL_OK);
        // 6. Set display offset to 0
        status = SSD1306_WriteCommand(dev, SSD1306_SET_DISPLAY_OFFSET);
        errNum += (status != HAL_OK);
        status = SSD1306_WriteCommand(dev, 0x00);
        errNum += (status != HAL_OK);
        // 7. Set display start line to 0
        status = SSD1306_WriteCommand(dev, SSD1306_SET_DISPLAY_START_LINE | 0x00);
        errNum += (status != HAL_OK);
        // 8. Segment re-map (should be changed if screen is horizontally mirrored)
        status = SSD1306_WriteCommand(dev, SSD1306_SET_SEGMENT_REMAP);
        errNum += (status != HAL_OK);
        // 9. COM output scan direction (should be changed if screen is vertically mirrored)
        status = SSD1306_WriteCommand(dev, SSD1306_SET_COM_REMAP);
        errNum += (status != HAL_OK);
        // 10. Set COM pins hardware configuration (p. 40, table 10-3 in datasheet)
        // Alternative COM pin configuration and Disable COM Left/Right re-map
        status = SSD1306_WriteCommand(dev, SSD1306_SET_COM_PINS_HW_CONFIG);
        errNum += (status != HAL_OK);
        status = SSD1306_WriteCommand(dev, 0x12);
        errNum += (status != HAL_OK);
        // 11. Set contrast control
        status = SSD1306_WriteCommand(dev, SSD1306_SET_CONTRAST);
        errNum += (status != HAL_OK);
        status = SSD1306_WriteCommand(dev, 0xFF);
        errNum += (status != HAL_OK);
        // 12. Disable entire display on (output follows RAM content)
        status = SSD1306_WriteCommand(dev, SSD1306_ENTIRE_DISPLAY_OFF);
        // 13. Set normal display mode
        status = SSD1306_WriteCommand(dev, SSD1306_SET_NORMAL_DISPLAY);
        errNum += (status != HAL_OK);
        // 14. Set Osc Frequency
        status = SSD1306_WriteCommand(dev, SSD1306_SET_DISPLAY_CLOCK_DIVIDE);
        errNum += (status != HAL_OK);
        status = SSD1306_WriteCommand(dev, 0xF0);
        errNum += (status != HAL_OK);
        // 15. Set pre-charge period
        status = SSD1306_WriteCommand(dev, SSD1306_SET_PRECHARGE_PERIOD);
        errNum += (status != HAL_OK);
        status = SSD1306_WriteCommand(dev, 0x22);
        errNum += (status != HAL_OK);
        // 16. Set Vcomh level to 0.77xVcc
        status = SSD1306_WriteCommand(dev, SSD1306_SET_VCOMH_DESELECT_LEVEL);
        errNum += (status != HAL_OK);
        status = SSD1306_WriteCommand(dev, 0x20);
        errNum += (status != HAL_OK);
        // 17. Enable charge pump regulator
        status = SSD1306_WriteCommand(dev, SSD1306_CHARGE_PUMP_SETTING);
        errNum += (status != HAL_OK);
        status = SSD1306_WriteCommand(dev, SSD1306_CHARGE_PUMP_ENABLED);
        errNum += (status != HAL_OK);
        // 18. Turn on the display
        status = SSD1306_WriteCommand(dev, SSD1306_DISPLAY_ON);
        errNum += (status != HAL_OK);
      }

      void updateScreen()
      {

      }
      
    private:
      ///@todo SSD1306 commands definitions
      static constexpr uint8_t ControlByteCommand = 0x00;
      static constexpr uint8_t ControlByteData = 0x40;
      // functions for cmd and data writing
      bool writeCommand(const uint8_t cmd)
      {
        auto& i2c = I2CinterfaceT::getInstance();
        using DisplayI2Cconfig = I2Cconfig<100000
                                  , 0x3C
                                  , SdaPinT
                                  , SclPinT>;
        i2c.configure<DisplayI2Cconfig>();
        const uint8_t transactionData[2] = {ControlByteCommand, cmd};
        i2c.write(reinterpret_cast<const std::byte*>(transactionData), sizeof(transactionData));
        return (i2c.getErrorStatus() == I2Cerror::NoError);
      }

      bool writeData(const std::byte* data, const size_t size)
      {
        auto& i2c = I2CinterfaceT::getInstance();
        using DisplayI2Cconfig = I2Cconfig<100000
                                  , 0x3C
                                  , SdaPinT
                                  , SclPinT>;
        i2c.configure<DisplayI2Cconfig>();
        const uint8_t transactionData[1] = {ControlByteData};
        i2c.write(reinterpret_cast<const std::byte*>(transactionData), sizeof(transactionData));
        return (i2c.getErrorStatus() == I2Cerror::NoError);
      }

      ///@todo add Canvas
  };
}

#endif //OLED_DISPLAY_HPP