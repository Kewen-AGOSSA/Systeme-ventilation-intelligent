#ifndef ILI9341_H
#define ILI9341_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf_lcd.h"

// // === Configuration écran ===
// #define ILI9341_WIDTH   240
// #define ILI9341_HEIGHT  320

// // === Broches SPI 
// #define ILI9341_SCK_PIN   29
// #define ILI9341_MOSI_PIN  25
// #define ILI9341_MISO_PIN  28
// #define ILI9341_SS_PIN    12
// #define ILI9341_DC_PIN    20


// #define ILI9341_SPI_INSTANCE 0

#ifdef __cplusplus
extern "C" {
#endif

// === Interface exposée à l'extérieur ===

/** Driver LCD pour l'API nrf_gfx */
extern const nrf_lcd_t nrf_lcd_ili9341;

/** Initialisation matérielle + écran */
extern ret_code_t ili9341_init(void);

/** Libération du périphérique SPI */
void ili9341_uninit(void);

/** Dessiner un pixel couleur 16 bits à (x,y) */
void ili9341_pixel_draw(uint16_t x, uint16_t y, uint32_t color);

/** Dessiner un rectangle plein */
void ili9341_rect_draw(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t color);

/** Affichage vide (nécessaire à l'API mais vide ici) */
void ili9341_dummy_display(void);

/** Rotation de l'affichage */
void ili9341_rotation_set(nrf_lcd_rotation_t rotation);

/** Inversion des couleurs (utile pour rétro-éclairage inversé) */
void ili9341_display_invert(bool invert);

#ifdef __cplusplus
}
#endif

#endif // ILI9341_H
