#ifndef OLED_DISPLAY_HPP
#define OLED_DISPLAY_HPP

#include <cstdint>
#include <cstddef>
#include <I2C.hpp>

#include <EmbeddedGfx/BufferedCanvas.hpp>


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
        using namespace Stm32;
        // initialize I2C
        auto& i2c = I2CinterfaceT::getInstance();
        using DisplayI2Cconfig = I2Cconfig<I2C_FREQUENCY
                                  , I2Caddress
                                  , SdaPinT
                                  , SclPinT>;
        i2c.template configure<DisplayI2Cconfig>();

        uint8_t cmdBuffer[3]{};
        uint8_t errNum = 0; // the number of error transactions
        bool status;
        // send commands for initialization
        // 1. Turn off the display
        cmdBuffer[0] = SSD1306_DISPLAY_OFF;
        status = writeCommand(cmdBuffer, 1);
        errNum += (status != true);
        // 2. Set MUX Ratio to 64
        cmdBuffer[0] = SSD1306_SET_MUX_RATIO;
        cmdBuffer[1] = 0x3F;  //< mux ratio is n + 1, write 63 for mux ratio 64
        status = writeCommand(cmdBuffer, 2);
        errNum += (status != true);
        // 3. Set horizontal addressing mode
        cmdBuffer[0] = SSD1306_SET_MEM_ADDR_MODE;
        cmdBuffer[1] = SSD1306_HORIZONTAL_ADDR_MODE;
        status = writeCommand(cmdBuffer, 2);
        errNum += (status != true);
        // 4. The column and page start and end addresses are fine for 128x64 display
        ///@todo make this changable based on display size
        cmdBuffer[0] = SSD1306_SET_PAGE_ADDR;
        cmdBuffer[1] = 0;
        cmdBuffer[2] = 7;
        status = writeCommand(cmdBuffer, 3);
        errNum += (status != true);
        cmdBuffer[0] = SSD1306_SET_COL_ADDR;
        cmdBuffer[1] = 0;
        cmdBuffer[2] = 127;
        status = writeCommand(cmdBuffer, 3);
        errNum += (status != true);
        // 5. Set display offset to 0
        cmdBuffer[0] = SSD1306_SET_DISPLAY_OFFSET;
        cmdBuffer[1] = 0x00;
        status = writeCommand(cmdBuffer, 2);
        errNum += (status != true);
        // 6. Set display start line to 0
        cmdBuffer[0] = SSD1306_SET_DISPLAY_START_LINE | 0x00;
        status = writeCommand(cmdBuffer, 1);
        errNum += (status != true);
        // 7. Segment re-map (should be changed if screen is horizontally mirrored)
        cmdBuffer[0] = SSD1306_SET_SEGMENT_REMAP;
        status = writeCommand(cmdBuffer, 1);
        errNum += (status != true);
        // 8. COM output scan direction (should be changed if screen is vertically mirrored)
        cmdBuffer[0] = SSD1306_SET_COM_REMAP;
        status = writeCommand(cmdBuffer, 1);
        errNum += (status != true);
        // 9. Set COM pins hardware configuration (p. 40, table 10-3 in datasheet)
        // Alternative COM pin configuration and Disable COM Left/Right re-map
        cmdBuffer[0] = SSD1306_SET_COM_PINS_HW_CONFIG;
        cmdBuffer[1] = 0x12;
        status = writeCommand(cmdBuffer, 2);
        errNum += (status != true);
        // 10. Set contrast control
        cmdBuffer[0] = SSD1306_SET_CONTRAST;
        cmdBuffer[1] = 0xFF;
        status = writeCommand(cmdBuffer, 2);
        errNum += (status != true);
        // 11. Disable entire display on (output follows RAM content)
        cmdBuffer[0] = SSD1306_ENTIRE_DISPLAY_OFF;
        status = writeCommand(cmdBuffer, 1);
        errNum += (status != true);
        // 12. Set normal display mode
        cmdBuffer[0] = SSD1306_SET_NORMAL_DISPLAY;
        status = writeCommand(cmdBuffer, 1);
        errNum += (status != true);
        // 13. Set Osc Frequency
        cmdBuffer[0] = SSD1306_SET_DISPLAY_CLOCK_DIVIDE;
        cmdBuffer[1] = 0xF0;
        status = writeCommand(cmdBuffer, 2);
        errNum += (status != true);
        // 14. Set pre-charge period
        cmdBuffer[0] = SSD1306_SET_PRECHARGE_PERIOD;
        cmdBuffer[1] = 0x22;
        status = writeCommand(cmdBuffer, 2);
        errNum += (status != true);
        // 15. Set Vcomh level to 0.77xVcc
        cmdBuffer[0] = SSD1306_SET_VCOMH_DESELECT_LEVEL;
        cmdBuffer[1] = 0x20;
        status = writeCommand(cmdBuffer, 2);
        errNum += (status != true);
        // 16. Enable charge pump regulator
        cmdBuffer[0] = SSD1306_CHARGE_PUMP_SETTING;
        cmdBuffer[1] = SSD1306_CHARGE_PUMP_ENABLED;
        status = writeCommand(cmdBuffer, 2);
        errNum += (status != true);
        // 17. Turn on the display
        cmdBuffer[0] = SSD1306_DISPLAY_ON;
        status = writeCommand(cmdBuffer, 1);
        errNum += (status != true);

        (void)errNum;
      }

      void updateScreen()
      {
        auto data = reinterpret_cast<const uint8_t*>(canvas_.getMatrix().data());
        writeData(data, canvas_.getHeight() * canvas_.getWidth() / canvas_.PageSize);
      }

      static constexpr uint8_t height_ = 64;
      static constexpr uint8_t width_ = 128;
      using CanvasT = EmbeddedGfx::BufferedCanvas<width_, height_, EmbeddedGfx::CanvasType::Page, EmbeddedGfx::ColorType::BlackAndWhite>;
      CanvasT& getCanvas() { return canvas_; }

    private:
      void resetColumnAndPageAddresses()
      {
        uint8_t cmdBuffer[3]{};
        bool status = 0;
        uint8_t errNum = 0;
        cmdBuffer[0] = SSD1306_SET_PAGE_ADDR;
        cmdBuffer[1] = 0;
        cmdBuffer[2] = 7;
        status = writeCommand(cmdBuffer, 3);
        errNum += (status != true);
        cmdBuffer[0] = SSD1306_SET_COL_ADDR;
        cmdBuffer[1] = 0;
        cmdBuffer[2] = 127;
        status = writeCommand(cmdBuffer, 3);
        errNum += (status != true);
      }
      // functions for cmd and data writing
      bool writeCommand(const uint8_t * const cmd, const size_t size)
      {
        using namespace Stm32;
        auto& i2c = I2CinterfaceT::getInstance();
        i2c.memWrite(reinterpret_cast<const std::byte*>(cmd), size, reinterpret_cast<const std::byte*>(&SSD1306_CONTROL_CMD_CONT), 1);
        return (i2c.getErrorStatus() == I2Cerror::NoError);
      }

      bool writeData(const uint8_t * const data, const size_t size)
      {
        using namespace Stm32;
        auto& i2c = I2CinterfaceT::getInstance();
        using DisplayI2Cconfig = I2Cconfig<I2C_FREQUENCY
                                  , I2Caddress
                                  , SdaPinT
                                  , SclPinT>;
        i2c.template configure<DisplayI2Cconfig>();
        resetColumnAndPageAddresses();
        i2c.memWrite(reinterpret_cast<const std::byte*>(data), size, reinterpret_cast<const std::byte*>(&SSD1306_CONTROL_DATA_CONT), 1);
        return (i2c.getErrorStatus() == I2Cerror::NoError);
      }
    private:
      CanvasT canvas_;

      /**************************************************
       * p. 20 in datasheet, refer to step 5
       * After initiating i2c communication
       * control byte is send:
       * C0 D/C# 0 0 0 0 0 0
       * C0 - if zero, no more control bytes are send,
       * 		and it allows continuous stream of data
       * 		to be sent
       * D/C# - if zero, command is send, otherwise data
       **************************************************/
      static constexpr uint8_t SSD1306_CONTROL_CMD_SINGLE		=	0x80;
      static constexpr uint8_t SSD1306_CONTROL_CMD_CONT			=	0x00;
      static constexpr uint8_t SSD1306_CONTROL_DATA_CONT		=	0x40;

      /************************************
       * starting with p. 28 in datasheet
       ************************************/
      // 1. Fundamental Command Table
      static constexpr uint8_t SSD1306_SET_CONTRAST				  =	0x81;
      static constexpr uint8_t SSD1306_ENTIRE_DISPLAY_OFF		=	0xA4;
      static constexpr uint8_t SSD1306_ENTIRE_DISPLAY_ON		=	0xA5;
      static constexpr uint8_t SSD1306_SET_NORMAL_DISPLAY		=	0xA6;
      static constexpr uint8_t SSD1306_SET_INVERSE_DISPLAY	=	0xA7;
      static constexpr uint8_t SSD1306_DISPLAY_ON 					=	0xAF;
      static constexpr uint8_t SSD1306_DISPLAY_OFF 				  =	0xAE;

      // 2. Scrolling Command Table
      static constexpr uint8_t SSD1306_RIGHT_HORIZONTAL_SCROLL		=		0x26;	// initiate setup for right horizontal scroll
      static constexpr uint8_t SSD1306_LEFT_HORIZONTAL_SCROLL			=	0x27;	// initiate setup for left horizontal scroll
      static constexpr uint8_t SSD1306_VERTICAL_RIGHT_HORIZONTAL_SCROLL	= 0x29;	// initiate setup for vertical and right horizontal scroll
      static constexpr uint8_t SSD1306_VERTICAL_LEFT_HORIZONTAL_SCROLL	=	0x2A;	// initiate setup for vertical and left horizontal scroll
      static constexpr uint8_t SSD1306_DEACTIVATE_SCROLL				=	0x2E;
      static constexpr uint8_t SSD1306_ACTIVATE_SCROLL					=	0x2F;
      static constexpr uint8_t SSD1306_VERTICAL_SCROLL_AREA			=	0xA3;	// initialize setup for vertical scroll area

      // 3. Addressing Setting Command Table
      static constexpr uint8_t SSD1306_SET_LOWER_COL_ADDR				=	0x00;	// should be ORed with value in range 0x0 - 0xF (refer datasheet, p. 34)
      static constexpr uint8_t SSD1306_SET_HIGHER_COL_ADDR			=	0x10;	// should be ORed with value in range 0x0 - 0xF (refer datasheet, p. 34)
      static constexpr uint8_t SSD1306_SET_MEM_ADDR_MODE				=	0x20;	// initiate memory addressing mode setup (refer p. 34-35 in datasheet)
      static constexpr uint8_t SSD1306_HORIZONTAL_ADDR_MODE			=	0x00;
      static constexpr uint8_t SSD1306_VERTICAL_ADDR_MODE				=	0x01;
      static constexpr uint8_t SSD1306_PAGE_ADDR_MODE					  =	0x10;
      static constexpr uint8_t SSD1306_SET_COL_ADDR				    	=	0x21;	// initiate column start and end address setup
      static constexpr uint8_t SSD1306_SET_PAGE_ADDR					  =	0x22;	// initiate page start and end address setup
      static constexpr uint8_t SSD1306_SET_PAGE_START_ADDR			= 0xB0;	// should be ORed with value in range 0x0 - 0x7 (refer datasheet, p. 36)

      // 4. Hardware Configuration (Panel resolution & layout related) Command Table
      static constexpr uint8_t SSD1306_SET_DISPLAY_START_LINE		=	0x40;
      static constexpr uint8_t SSD1306_SET_SEGMENT_NOREMAP			=	0xA0;
      static constexpr uint8_t SSD1306_SET_SEGMENT_REMAP				=	0xA1;
      static constexpr uint8_t SSD1306_SET_MUX_RATIO						= 0xA8;	// initiate mux ratio setup
      static constexpr uint8_t SSD1306_SET_COM_NOREMAP					=	0xC0;
      static constexpr uint8_t SSD1306_SET_COM_REMAP						= 0xC8;
      static constexpr uint8_t SSD1306_SET_DISPLAY_OFFSET				=	0xD3;	// initiate display offset setup
      static constexpr uint8_t SSD1306_SET_COM_PINS_HW_CONFIG		=	0xDA;	// initiate com pin configuration setup

      // 5. Timing & Driving Scheme Setting Command Table
      static constexpr uint8_t SSD1306_SET_DISPLAY_CLOCK_DIVIDE	 =	0xD5;	// initiate clock divide ratio setup
      static constexpr uint8_t SSD1306_SET_PRECHARGE_PERIOD			 =	0xD9;	// initiate precharge period setup
      static constexpr uint8_t SSD1306_SET_VCOMH_DESELECT_LEVEL	 =	0xDB;	// initiate Vcomh deselect level setup
      static constexpr uint8_t SSD1306_NOP								       =	0xE3;

      // Charge pump setting
      static constexpr uint8_t SSD1306_CHARGE_PUMP_SETTING			=	0x8D;
      static constexpr uint8_t SSD1306_CHARGE_PUMP_ENABLED			=	0x14;
      static constexpr uint8_t SSD1306_CHARGE_PUMP_DISABLED			=	0x10;

      // I2C settings
      static constexpr uint32_t I2C_FREQUENCY = 100000U; // Hz

  };
}

#endif //OLED_DISPLAY_HPP