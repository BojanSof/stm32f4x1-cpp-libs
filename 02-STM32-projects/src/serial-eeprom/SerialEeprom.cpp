#include <Gpio.hpp>
#include <Clock.hpp>
#include <CycleCounter.hpp>
#include <I2C.hpp>

int main()
{
  using namespace Stm32;
  
  deviceInit();
  auto& i2cInterface = I2C<1>::getInstance();
  // I2Cconfig<100000U, 0x50, Pins::PB7, Pins::PB6> i2cConfig;
  // i2cInterface.configure(i2cConfig);
  // const uint8_t arrWrite[2] = {0x00, 0x00};
  // i2cInterface.write(reinterpret_cast<const std::byte*>(&arrWrite[0]), 2);
  // uint8_t varRead;
  // i2cInterface.read(reinterpret_cast<std::byte*>(&varRead), 1);

  Pins::PB7 sda;
  Pins::PB6 scl;
  sda.setAlternateFunction(Pins::PB7::AlternateFunctions::I2C1_SDA);
  scl.setAlternateFunction(Pins::PB6::AlternateFunctions::I2C1_SCL);
  sda.setOutputType(GpioOutputType::OpenDrain);
  scl.setOutputType(GpioOutputType::OpenDrain);
  // RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  // I2C1->CR2 &= ~(0x3F << I2C_CR2_FREQ_Pos);
  // I2C1->CR2 |= (Pclk1Frequency / 1000000U) << I2C_CR2_FREQ_Pos;
  // I2C1->FLTR &= ~(0xF << I2C_FLTR_DNF_Pos);
  // I2C1->FLTR |= 0xF << I2C_FLTR_DNF_Pos;
  // I2C1->CR2 |= I2C_CR2_ITBUFEN;
  // I2C1->CR2 |= I2C_CR2_ITEVTEN;
  // I2C1->CR2 |= I2C_CR2_ITERREN;

  I2C1->CCR &= ~(0xFFF << I2C_CCR_CCR_Pos);
  I2C1->TRISE &= ~(0x3F << I2C_TRISE_TRISE_Pos);

  // standard mode
  I2C1->CCR &= ~I2C_CCR_FS;
  I2C1->CCR |= (Pclk1Frequency / (2 * 100000U)) << I2C_CCR_CCR_Pos;

  // configure rise time register (for sm, 1000ns)
  I2C1->TRISE |= 43 << I2C_TRISE_TRISE_Pos;

  // automate sending ACK
  // I2C1->CR1 |= I2C_CR1_ACK;
  
  // enable I2C peripheral
  I2C1->CR1 |= I2C_CR1_PE;
  
  // generate start condition
  I2C1->CR1 |= I2C_CR1_START;
  // wait until the start condition is generated
  while(!(I2C1->SR1 & I2C_SR1_SB));

  // send slave address
  I2C1->DR = 0xa0;
  // wait until the slave address is sent
  while(!(I2C1->SR1 & I2C_SR1_ADDR));
  // sequential read of SR1 and SR2
  (void)I2C1->SR2;
  
  // two write cycles for addresing the memory
  I2C1->DR = 0b00000000;
  while(!(I2C1->SR1 & I2C_SR1_TXE));
  I2C1->DR = 0b00000000;
  while(!(I2C1->SR1 & I2C_SR1_TXE));

  // generate start condition
  I2C1->CR1 |= I2C_CR1_START;
  // wait until the start condition is generated
  while(!(I2C1->SR1 & I2C_SR1_SB));

  // send slave address
  I2C1->DR = 0xa1;
  // wait until the slave address is sent
  while(!(I2C1->SR1 & I2C_SR1_ADDR));
  // sequential read of SR1 and SR2
  (void)I2C1->SR2;

  uint8_t data = I2C1->DR;
  (void)data;

  while(true)
  {
    
  }

  return 0;
}
