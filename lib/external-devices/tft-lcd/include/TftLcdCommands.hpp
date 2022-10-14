#ifndef TFT_LCD_COMMANDS_HPP
#define TFT_LCD_COMMANDS_HPP

namespace Devices
{
  namespace TftLcdCommands
  {
    // commands
    static constexpr uint8_t NOP                                = 0x00;
    static constexpr uint8_t SoftReset                          = 0x01;
    static constexpr uint8_t ReadId                             = 0x04;
    static constexpr uint8_t ReadNumDsiErrors                   = 0x05;
    static constexpr uint8_t ReadDisplayStatus                  = 0x09;
    static constexpr uint8_t ReadDisplayPowerMode               = 0x0A;
    static constexpr uint8_t ReadDisplayMemoryAccessControl     = 0x0B;
    static constexpr uint8_t ReadDisplayPixelFormat             = 0x0C;
    static constexpr uint8_t ReadDisplayImageMode               = 0x0D;
    static constexpr uint8_t ReadDisplaySignalMode              = 0x0E;
    static constexpr uint8_t ReadDisplaySelfDiagnosticResult    = 0x0F;
    static constexpr uint8_t SleepIn                            = 0x10;
    static constexpr uint8_t SleepOut                           = 0x11;
    static constexpr uint8_t PartialModeOn                      = 0x12;
    static constexpr uint8_t NormalDisplayModeOn                = 0x13;
    static constexpr uint8_t DisplayInversionOff                = 0x20;
    static constexpr uint8_t DisplayInversionOn                 = 0x21;
    static constexpr uint8_t AllPixelsOff                       = 0x22;
    static constexpr uint8_t AllPixelsOn                        = 0x23;
    static constexpr uint8_t DisplayOff                         = 0x28;
    static constexpr uint8_t DisplayOn                          = 0x29;
    static constexpr uint8_t ColumnAddressSet                   = 0x2A;
    static constexpr uint8_t PageAddressSet                     = 0x2B;
    static constexpr uint8_t MemoryWrite                        = 0x2C;
    static constexpr uint8_t MemoryRead                         = 0x2E;
    static constexpr uint8_t MemoryAccessControl                = 0x36;
    static constexpr uint8_t IdleModeOff                        = 0x38;
    static constexpr uint8_t IdleModeOn                         = 0x39;
    static constexpr uint8_t InterfacePixelFormat               = 0x3A;
    static constexpr uint8_t MemoryWriteContinue                = 0x3C;
    static constexpr uint8_t MemoryReadContinue                 = 0x3E;
    static constexpr uint8_t WriteDisplayBrightnessValue        = 0x51;
    static constexpr uint8_t ReadDisplayBrightnessValue         = 0x52;
    static constexpr uint8_t InterfaceModeControl               = 0xB0;
    static constexpr uint8_t FrameRateControlNormalMode         = 0xB1;
    static constexpr uint8_t FrameRateControlIdleMode           = 0xB2;
    static constexpr uint8_t FrameRateControlPartialMode        = 0xB3;
    static constexpr uint8_t DisplayInversion                   = 0xB4;
    static constexpr uint8_t DisplayFunctionControl             = 0xB6;
    static constexpr uint8_t EntryModeSet                       = 0xB7;
    static constexpr uint8_t HsLanesControl                     = 0xBE;
    static constexpr uint8_t PowerControl1                      = 0xC0;
    static constexpr uint8_t PowerControl2                      = 0xC1;
    static constexpr uint8_t PowerControl3                      = 0xC2;
    static constexpr uint8_t PowerControl4                      = 0xC3;
    static constexpr uint8_t PowerControl5                      = 0xC4;
    static constexpr uint8_t VcomControl                        = 0xC5;
    static constexpr uint8_t NVMemoryWrite                      = 0xD0;
    static constexpr uint8_t NVMemoryProtectionKey              = 0xD1;
    static constexpr uint8_t NVMemoryStatusRead                 = 0xD2;
    static constexpr uint8_t ReadId4                            = 0xD3;
    static constexpr uint8_t AdjustControl1                     = 0xD7;
    static constexpr uint8_t ReadIdVersion                      = 0xD8;
    static constexpr uint8_t ReadId1                            = 0xDA;
    static constexpr uint8_t ReadId2                            = 0xDB;
    static constexpr uint8_t ReadId3                            = 0xDC;
    static constexpr uint8_t PositiveGammaControl               = 0xE0;
    static constexpr uint8_t NegativeGammaCorrection            = 0xE1;
    static constexpr uint8_t DigitalGammaControl1               = 0xE2;
    static constexpr uint8_t DigitalGammaControl2               = 0xE3;
    static constexpr uint8_t SetImageFunction                   = 0xE9;
    static constexpr uint8_t AdjustControl2                     = 0xF2;
    static constexpr uint8_t AdjustControl3                     = 0xF7;
    static constexpr uint8_t AdjustControl4                     = 0xF8;
    static constexpr uint8_t AdjustControl5                     = 0xF9;
    static constexpr uint8_t AdjustControl6                     = 0xFC;
    static constexpr uint8_t AdjustControl7                     = 0xFF;
    
    // commands' parameters
    static constexpr uint8_t MemoryAccessControlMY    = 1 << 7;
    static constexpr uint8_t MemoryAccessControlMX    = 1 << 6;
    static constexpr uint8_t MemoryAccessControlMV    = 1 << 5;
    static constexpr uint8_t MemoryAccessControlML    = 1 << 4;
    static constexpr uint8_t MemoryAccessControlBGR   = 1 << 3;
    static constexpr uint8_t MemoryAccessControlMH    = 1 << 2;
  }
}

#endif //TFT_LCD_COMMANDS_HPP