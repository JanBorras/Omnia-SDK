#include "omnia_port.h"
#include "st7735.h"

// Tria plataforma a l’enllaçat
void omnia_port_stm32_init(void);

int main(void){
  omnia_port_stm32_init();               // registra el port
  omnia_port_t* P = omnia_port_get();

  st7735_t lcd = {
    .port = P,
    .spi  = /* handle SPI */,
    .pin_cs  = /* pin */,
    .pin_dc  = /* pin */,
    .pin_rst = /* pin */,
    .width = 128, .height = 160
  };

  st7735_init(&lcd);
  st7735_fill(&lcd, 0x0000);              // negre
  // ...
  for(;;){}
}
