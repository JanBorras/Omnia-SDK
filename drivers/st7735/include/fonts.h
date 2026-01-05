// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (c) 2025 Jan Borràs Ros
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file fonts.h
 * @brief Fixed-width 5x8 bitmap font table (ASCII).
 *
 * This header declares the public interface for a compact bitmap font
 * used by lightweight display drivers (e.g. ST7735).
 *
 * Font characteristics:
 * - Fixed width: 5 columns per character
 * - Fixed height: 8 rows per character (encoded bitwise per column)
 * - ASCII range: characters 0x20 (' ') to 0x7F
 *
 * Storage format:
 * - Each glyph is stored column-wise.
 * - Each column is a byte, where bit 0 is the top pixel and bit 7 the bottom.
 *
 * Design rationale:
 * - Minimal memory footprint.
 * - Cache-friendly sequential access.
 * - Suitable for MCUs without dynamic allocation or font rendering engines.
 *
 * The actual bitmap data is defined in the corresponding implementation file.
 */

#ifndef FONT_H
#define FONT_H

#include <stdint.h>

/**
 * @brief Number of columns per character glyph.
 *
 * All glyphs in @ref FONTS use a fixed width of 5 pixels.
 */
#define CHARS_COLS_LENGTH  5

/**
 * @brief Bitmap font table for ASCII characters.
 *
 * Indexing:
 * - Character code `c` maps to `FONTS[c - 32]`
 * - Valid for `c` in the range [32, 127]
 *
 * Each entry contains @ref CHARS_COLS_LENGTH bytes representing
 * the vertical pixel columns of the glyph.
 */
extern const uint8_t FONTS[][CHARS_COLS_LENGTH];

#endif /* FONT_H */
