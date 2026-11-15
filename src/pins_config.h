#ifdef ARDUINO_ESP32P4_DEV

#pragma once

// Pin names used by this driver.  The numbers themselves live in
// JC8012P4A1C_Board (chipguy_JC8012P4A1C_board), the one home for the board's
// pin map; these are aliases so the driver code reads as it always has.
#include <JC8012P4A1C_Board.h>

#define LCD_H_RES 800
#define LCD_V_RES 1280

#define LCD_RST (JC8012P4A1C_Board::LCD_RST_PIN)
#define LCD_LED (JC8012P4A1C_Board::LCD_BACKLIGHT_PIN)

#define TP_RST  (JC8012P4A1C_Board::TOUCH_RST_PIN)
#define TP_INT  (JC8012P4A1C_Board::TOUCH_INT_PIN)
// Touch SDA/SCL are the board's shared I2C bus (board.Wire), not ours to name.

#endif // ARDUINO_ESP32P4_DEV
