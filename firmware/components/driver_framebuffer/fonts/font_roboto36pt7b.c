#include "../include/driver_framebuffer.h"
const uint8_t roboto36pt7bBitmaps[] = {
  0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0x70,
  0x00, 0x07, 0xFF, 0x70, 0xC7, 0xC7, 0xC7, 0xC7, 0xC7, 0xC6, 0xC6, 0xC6,
  0xC6, 0xC6, 0x00, 0x70, 0x70, 0x01, 0x81, 0x80, 0x0E, 0x0E, 0x00, 0x38,
  0x38, 0x00, 0xE0, 0xE0, 0x03, 0x83, 0x80, 0x0C, 0x0C, 0x00, 0x70, 0x70,
  0x3F, 0xFF, 0xFC, 0xFF, 0xFF, 0xF3, 0xFF, 0xFF, 0xC0, 0x70, 0x70, 0x01,
  0x83, 0x80, 0x0E, 0x0E, 0x00, 0x38, 0x38, 0x00, 0xE0, 0xE0, 0x03, 0x83,
  0x80, 0x0C, 0x0C, 0x0F, 0xFF, 0xFF, 0xBF, 0xFF, 0xFE, 0xFF, 0xFF, 0xF8,
  0x1C, 0x1C, 0x00, 0x70, 0x60, 0x03, 0x83, 0x80, 0x0E, 0x0E, 0x00, 0x38,
  0x38, 0x00, 0xE0, 0xE0, 0x03, 0x83, 0x00, 0x1C, 0x1C, 0x00, 0x00, 0xE0,
  0x00, 0x38, 0x00, 0x0E, 0x00, 0x03, 0x80, 0x03, 0xF8, 0x03, 0xFF, 0x81,
  0xFF, 0xF0, 0xF8, 0x3E, 0x3C, 0x07, 0x9E, 0x00, 0xF7, 0x80, 0x3D, 0xE0,
  0x0F, 0x78, 0x03, 0xDE, 0x00, 0x03, 0xC0, 0x00, 0xF8, 0x00, 0x1F, 0x80,
  0x03, 0xF8, 0x00, 0x7F, 0xC0, 0x07, 0xF8, 0x00, 0x3F, 0x80, 0x03, 0xE0,
  0x00, 0x3C, 0x00, 0x0F, 0xF0, 0x01, 0xFC, 0x00, 0x7F, 0x00, 0x1D, 0xC0,
  0x0F, 0x78, 0x03, 0xDF, 0x83, 0xE3, 0xFF, 0xF8, 0x7F, 0xF8, 0x07, 0xF8,
  0x00, 0x38, 0x00, 0x0E, 0x00, 0x03, 0x80, 0x00, 0xE0, 0x00, 0x1F, 0x00,
  0x00, 0x0F, 0xF0, 0x00, 0x07, 0x0E, 0x00, 0x03, 0x83, 0x80, 0x80, 0xE0,
  0x70, 0x38, 0x38, 0x1C, 0x1C, 0x0E, 0x07, 0x0E, 0x03, 0x81, 0xC3, 0x80,
  0xE0, 0x71, 0xC0, 0x38, 0x38, 0xE0, 0x07, 0x0E, 0x38, 0x00, 0xFF, 0x1C,
  0x00, 0x1F, 0x0E, 0x00, 0x00, 0x03, 0x80, 0x00, 0x01, 0xC0, 0x00, 0x00,
  0x60, 0x00, 0x00, 0x38, 0x7C, 0x00, 0x1C, 0x3F, 0xC0, 0x07, 0x1C, 0x78,
  0x03, 0x8E, 0x0E, 0x01, 0xC3, 0x81, 0x80, 0x70, 0xE0, 0x70, 0x38, 0x38,
  0x1C, 0x1C, 0x0E, 0x07, 0x07, 0x03, 0x81, 0x80, 0x80, 0xE0, 0xE0, 0x00,
  0x1C, 0x78, 0x00, 0x03, 0xFC, 0x00, 0x00, 0x7C, 0x00, 0x03, 0xF0, 0x00,
  0x0F, 0xF8, 0x00, 0x3F, 0xF8, 0x00, 0xF8, 0xF8, 0x01, 0xE0, 0xF0, 0x07,
  0x80, 0xE0, 0x0F, 0x01, 0xC0, 0x1E, 0x03, 0x80, 0x1C, 0x0F, 0x00, 0x3C,
  0x3C, 0x00, 0x7D, 0xF0, 0x00, 0x7F, 0xC0, 0x00, 0x7F, 0x00, 0x00, 0xF8,
  0x00, 0x03, 0xF8, 0x00, 0x0F, 0xF8, 0x00, 0x3C, 0xF8, 0x38, 0xF0, 0xF0,
  0x73, 0xC0, 0xF1, 0xE7, 0x80, 0xF3, 0xCF, 0x00, 0xF7, 0x1E, 0x01, 0xFE,
  0x3C, 0x01, 0xF8, 0x78, 0x01, 0xF0, 0x78, 0x03, 0xE0, 0xFC, 0x1F, 0xE0,
  0xFF, 0xFD, 0xE0, 0xFF, 0xF3, 0xE0, 0x3F, 0x83, 0xE0, 0xFF, 0xFF, 0xFF,
  0xE0, 0x00, 0x80, 0x70, 0x3C, 0x1E, 0x0F, 0x03, 0x81, 0xC0, 0x70, 0x38,
  0x0E, 0x07, 0x81, 0xC0, 0x70, 0x3C, 0x0F, 0x03, 0xC0, 0xE0, 0x38, 0x0E,
  0x03, 0x80, 0xE0, 0x38, 0x0E, 0x03, 0x80, 0xE0, 0x3C, 0x0F, 0x03, 0xC0,
  0x70, 0x1C, 0x07, 0x80, 0xE0, 0x38, 0x07, 0x01, 0xC0, 0x38, 0x0F, 0x01,
  0xE0, 0x3C, 0x07, 0x00, 0x80, 0x80, 0x38, 0x0F, 0x01, 0xC0, 0x38, 0x07,
  0x01, 0xE0, 0x38, 0x0F, 0x01, 0xC0, 0x70, 0x1E, 0x07, 0x80, 0xE0, 0x38,
  0x0F, 0x03, 0xC0, 0xF0, 0x3C, 0x0F, 0x03, 0xC0, 0xF0, 0x3C, 0x0F, 0x03,
  0xC0, 0xF0, 0x3C, 0x0E, 0x03, 0x81, 0xE0, 0x70, 0x1C, 0x0F, 0x03, 0x81,
  0xE0, 0x70, 0x38, 0x1C, 0x0F, 0x03, 0x80, 0x80, 0x00, 0x03, 0x80, 0x03,
  0x80, 0x03, 0x80, 0x03, 0x80, 0x03, 0x80, 0xE3, 0x86, 0xFB, 0xBE, 0xFF,
  0xFF, 0x3F, 0xF8, 0x07, 0xC0, 0x07, 0xC0, 0x0E, 0xE0, 0x1E, 0xF0, 0x3C,
  0x78, 0x38, 0x38, 0x18, 0x3C, 0x10, 0x20, 0x01, 0xE0, 0x00, 0x3C, 0x00,
  0x07, 0x80, 0x00, 0xF0, 0x00, 0x1E, 0x00, 0x03, 0xC0, 0x00, 0x78, 0x00,
  0x0F, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0xF0, 0x00,
  0x1E, 0x00, 0x03, 0xC0, 0x00, 0x78, 0x00, 0x0F, 0x00, 0x01, 0xE0, 0x00,
  0x3C, 0x00, 0x07, 0x80, 0x00, 0xF0, 0x00, 0x7B, 0xDE, 0xF7, 0xBD, 0xDE,
  0xE2, 0x00, 0xFF, 0xFF, 0xFF, 0xE0, 0xFF, 0xFF, 0x00, 0x0E, 0x00, 0x1C,
  0x00, 0x70, 0x00, 0xE0, 0x03, 0xC0, 0x07, 0x00, 0x0E, 0x00, 0x38, 0x00,
  0x70, 0x00, 0xE0, 0x03, 0x80, 0x07, 0x00, 0x1E, 0x00, 0x38, 0x00, 0x70,
  0x01, 0xC0, 0x03, 0x80, 0x07, 0x00, 0x1C, 0x00, 0x38, 0x00, 0xE0, 0x01,
  0xC0, 0x03, 0x80, 0x0E, 0x00, 0x1C, 0x00, 0x78, 0x00, 0xE0, 0x01, 0xC0,
  0x07, 0x00, 0x0E, 0x00, 0x3C, 0x00, 0x00, 0x03, 0xF8, 0x03, 0xFF, 0x81,
  0xFF, 0xF0, 0xF8, 0x3C, 0x3C, 0x07, 0x9E, 0x01, 0xE7, 0x80, 0x3D, 0xC0,
  0x0F, 0xF0, 0x03, 0xFC, 0x00, 0xFF, 0x00, 0x1F, 0xC0, 0x07, 0xF0, 0x01,
  0xFC, 0x00, 0x7F, 0x00, 0x1F, 0xC0, 0x07, 0xF0, 0x01, 0xFC, 0x00, 0x7F,
  0x00, 0x1F, 0xC0, 0x0F, 0xF0, 0x03, 0xDC, 0x00, 0xF7, 0x80, 0x3D, 0xE0,
  0x1E, 0x3C, 0x07, 0x8F, 0x83, 0xC1, 0xFF, 0xF0, 0x3F, 0xF0, 0x03, 0xF8,
  0x00, 0x00, 0x60, 0x3C, 0x3F, 0x9F, 0xFF, 0xEF, 0xF1, 0xF0, 0x38, 0x07,
  0x00, 0xE0, 0x1C, 0x03, 0x80, 0x70, 0x0E, 0x01, 0xC0, 0x38, 0x07, 0x00,
  0xE0, 0x1C, 0x03, 0x80, 0x70, 0x0E, 0x01, 0xC0, 0x38, 0x07, 0x00, 0xE0,
  0x1C, 0x03, 0x80, 0x70, 0x0E, 0x03, 0xF8, 0x01, 0xFF, 0xC0, 0xFF, 0xFC,
  0x3E, 0x0F, 0xC7, 0x80, 0x79, 0xE0, 0x0F, 0x3C, 0x00, 0xF7, 0x80, 0x1E,
  0xE0, 0x03, 0xC0, 0x00, 0x70, 0x00, 0x0E, 0x00, 0x03, 0xC0, 0x00, 0xF0,
  0x00, 0x1E, 0x00, 0x07, 0x80, 0x01, 0xE0, 0x00, 0x78, 0x00, 0x1F, 0x00,
  0x07, 0xC0, 0x01, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00,
  0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFE, 0x03, 0xF0, 0x03, 0xFF, 0x03, 0xFF, 0xE1, 0xF0, 0x7C, 0x78,
  0x07, 0xBC, 0x01, 0xEF, 0x00, 0x3B, 0xC0, 0x0E, 0x00, 0x03, 0x80, 0x00,
  0xE0, 0x00, 0x78, 0x00, 0x3C, 0x00, 0x1F, 0x00, 0xFF, 0x80, 0x3F, 0x80,
  0x0F, 0xF8, 0x00, 0x1F, 0x00, 0x01, 0xE0, 0x00, 0x38, 0x00, 0x0F, 0x00,
  0x03, 0xF8, 0x00, 0xFF, 0x00, 0x3F, 0xC0, 0x0F, 0x78, 0x07, 0x9F, 0x03,
  0xE3, 0xFF, 0xF0, 0x7F, 0xF0, 0x07, 0xF0, 0x00, 0x00, 0x0F, 0x80, 0x00,
  0x7C, 0x00, 0x07, 0xE0, 0x00, 0x7F, 0x00, 0x03, 0xF8, 0x00, 0x3F, 0xC0,
  0x01, 0xDE, 0x00, 0x1E, 0xF0, 0x01, 0xE7, 0x80, 0x0E, 0x3C, 0x00, 0xF1,
  0xE0, 0x0F, 0x0F, 0x00, 0x70, 0x78, 0x07, 0x83, 0xC0, 0x78, 0x1E, 0x03,
  0x80, 0xF0, 0x3C, 0x07, 0x81, 0xC0, 0x3C, 0x1C, 0x01, 0xE1, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x1E, 0x00, 0x00, 0xF0, 0x00,
  0x07, 0x80, 0x00, 0x3C, 0x00, 0x01, 0xE0, 0x00, 0x0F, 0x00, 0x00, 0x78,
  0x00, 0x3F, 0xFF, 0x8F, 0xFF, 0xE3, 0xFF, 0xF8, 0xE0, 0x00, 0x38, 0x00,
  0x0E, 0x00, 0x03, 0x80, 0x00, 0xE0, 0x00, 0x38, 0x00, 0x1E, 0x00, 0x07,
  0xBF, 0x01, 0xFF, 0xF0, 0x7F, 0xFF, 0x1F, 0x07, 0xC3, 0x80, 0xF8, 0x00,
  0x1E, 0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03,
  0xF8, 0x00, 0xFF, 0x00, 0x3F, 0xC0, 0x1E, 0x78, 0x07, 0x9F, 0x07, 0xC3,
  0xFF, 0xE0, 0x7F, 0xF0, 0x03, 0xF0, 0x00, 0x00, 0x7C, 0x00, 0xFF, 0x00,
  0x7F, 0xC0, 0x3F, 0x00, 0x1E, 0x00, 0x0F, 0x00, 0x07, 0x80, 0x01, 0xE0,
  0x00, 0x70, 0x00, 0x3C, 0x00, 0x0F, 0x3F, 0x03, 0xBF, 0xF0, 0xFF, 0xFE,
  0x3F, 0x07, 0xCF, 0x80, 0xFB, 0xC0, 0x1E, 0xE0, 0x07, 0xB8, 0x00, 0xEE,
  0x00, 0x3F, 0x80, 0x0F, 0xE0, 0x03, 0xFC, 0x00, 0xEF, 0x00, 0x79, 0xE0,
  0x1E, 0x78, 0x0F, 0x0F, 0x87, 0xC1, 0xFF, 0xE0, 0x3F, 0xF0, 0x03, 0xF0,
  0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x0E, 0x00,
  0x01, 0xE0, 0x00, 0x1C, 0x00, 0x03, 0xC0, 0x00, 0x38, 0x00, 0x07, 0x80,
  0x00, 0x70, 0x00, 0x0F, 0x00, 0x00, 0xF0, 0x00, 0x0E, 0x00, 0x01, 0xE0,
  0x00, 0x1C, 0x00, 0x03, 0xC0, 0x00, 0x38, 0x00, 0x07, 0x80, 0x00, 0x78,
  0x00, 0x0F, 0x00, 0x00, 0xF0, 0x00, 0x0E, 0x00, 0x01, 0xE0, 0x00, 0x1C,
  0x00, 0x03, 0xC0, 0x00, 0x38, 0x00, 0x07, 0x80, 0x00, 0x78, 0x00, 0x0F,
  0x00, 0x00, 0x03, 0xF8, 0x03, 0xFF, 0x01, 0xFF, 0xF0, 0xF8, 0x7C, 0x7C,
  0x07, 0x9E, 0x01, 0xE7, 0x80, 0x39, 0xC0, 0x0F, 0x70, 0x03, 0xDE, 0x00,
  0xE7, 0x80, 0x78, 0xF0, 0x1E, 0x3E, 0x1F, 0x07, 0xFF, 0x80, 0x7F, 0x80,
  0x7F, 0xF8, 0x3E, 0x0F, 0x1E, 0x01, 0xE7, 0x00, 0x3F, 0xC0, 0x0F, 0xF0,
  0x01, 0xFC, 0x00, 0x7F, 0x00, 0x3F, 0xC0, 0x0F, 0x78, 0x07, 0xDF, 0x83,
  0xE3, 0xFF, 0xF0, 0x7F, 0xF8, 0x03, 0xF8, 0x00, 0x03, 0xF0, 0x03, 0xFF,
  0x03, 0xFF, 0xE0, 0xF8, 0x7C, 0x78, 0x0F, 0x1C, 0x01, 0xEF, 0x00, 0x7B,
  0xC0, 0x0E, 0xF0, 0x03, 0xFC, 0x00, 0xFF, 0x00, 0x3F, 0xC0, 0x0F, 0xF0,
  0x03, 0xDC, 0x01, 0xF7, 0x80, 0x7D, 0xF0, 0x7F, 0x3F, 0xFF, 0xC7, 0xFE,
  0xF0, 0x7E, 0x38, 0x00, 0x0E, 0x00, 0x07, 0x80, 0x01, 0xE0, 0x00, 0xF0,
  0x00, 0x3C, 0x00, 0x3E, 0x00, 0x3F, 0x00, 0xFF, 0x80, 0x3F, 0x80, 0x0F,
  0x80, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0x7B, 0xDE, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1E, 0xF7, 0xBD, 0xEF, 0x77, 0xB8, 0x80, 0x00, 0x01, 0x00, 0x07, 0x00,
  0x1F, 0x00, 0x7F, 0x01, 0xFE, 0x07, 0xF8, 0x1F, 0xC0, 0xFF, 0x00, 0xF8,
  0x00, 0xF8, 0x00, 0x7F, 0x00, 0x1F, 0xC0, 0x07, 0xF8, 0x01, 0xFE, 0x00,
  0x7F, 0x00, 0x1F, 0x00, 0x07, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0xE0, 0x00, 0xF8, 0x00, 0xFE,
  0x00, 0x7F, 0xC0, 0x0F, 0xF0, 0x03, 0xFC, 0x00, 0x7F, 0x00, 0x1F, 0x00,
  0x1F, 0x00, 0x7F, 0x03, 0xFC, 0x0F, 0xF0, 0x7F, 0xC0, 0xFE, 0x00, 0xF8,
  0x00, 0xE0, 0x00, 0x80, 0x00, 0x07, 0xF0, 0x1F, 0xFC, 0x3F, 0xFE, 0x7C,
  0x1E, 0x78, 0x0F, 0x70, 0x0F, 0xF0, 0x07, 0x00, 0x07, 0x00, 0x07, 0x00,
  0x0F, 0x00, 0x0F, 0x00, 0x1E, 0x00, 0x3E, 0x00, 0x7C, 0x00, 0xF8, 0x01,
  0xF0, 0x01, 0xE0, 0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x03, 0xC0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80, 0x03, 0xC0, 0x03,
  0xC0, 0x03, 0x80, 0x00, 0x07, 0xF8, 0x00, 0x00, 0x3F, 0xFF, 0x00, 0x00,
  0xFF, 0xFF, 0xC0, 0x01, 0xF8, 0x07, 0xE0, 0x03, 0xE0, 0x00, 0xF0, 0x07,
  0x80, 0x00, 0x78, 0x0F, 0x00, 0x00, 0x3C, 0x1E, 0x00, 0x00, 0x1C, 0x1C,
  0x00, 0x00, 0x0E, 0x38, 0x01, 0xF0, 0x0E, 0x38, 0x07, 0xFC, 0x06, 0x38,
  0x0F, 0xFE, 0x06, 0x70, 0x1F, 0x0E, 0x07, 0x70, 0x1C, 0x0E, 0x07, 0x70,
  0x38, 0x0E, 0x07, 0x60, 0x38, 0x0C, 0x07, 0xE0, 0x78, 0x1C, 0x07, 0xE0,
  0x70, 0x1C, 0x07, 0xE0, 0x70, 0x1C, 0x07, 0xE0, 0x70, 0x1C, 0x07, 0xE0,
  0x70, 0x1C, 0x07, 0xE0, 0x70, 0x1C, 0x06, 0xE0, 0x70, 0x3C, 0x0E, 0x60,
  0x70, 0x3C, 0x0E, 0x70, 0x78, 0x7C, 0x1C, 0x70, 0x3F, 0xEF, 0xF8, 0x70,
  0x3F, 0xCF, 0xF0, 0x38, 0x0F, 0x07, 0xE0, 0x3C, 0x00, 0x00, 0x00, 0x1C,
  0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x04, 0x00, 0x07,
  0xF0, 0x1C, 0x00, 0x03, 0xFF, 0xFE, 0x00, 0x00, 0xFF, 0xFC, 0x00, 0x00,
  0x1F, 0xE0, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x07, 0x80, 0x00, 0x03, 0xF0,
  0x00, 0x00, 0xFC, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x1F, 0xE0, 0x00, 0x07,
  0x38, 0x00, 0x01, 0xCE, 0x00, 0x00, 0xF3, 0xC0, 0x00, 0x38, 0x70, 0x00,
  0x1E, 0x1E, 0x00, 0x07, 0x87, 0x80, 0x01, 0xC0, 0xE0, 0x00, 0xF0, 0x3C,
  0x00, 0x38, 0x07, 0x00, 0x0E, 0x01, 0xC0, 0x07, 0x80, 0x78, 0x01, 0xC0,
  0x0E, 0x00, 0xFF, 0xFF, 0xC0, 0x3F, 0xFF, 0xF0, 0x0F, 0xFF, 0xFC, 0x07,
  0x80, 0x07, 0x81, 0xE0, 0x01, 0xE0, 0x70, 0x00, 0x38, 0x3C, 0x00, 0x0F,
  0x0F, 0x00, 0x03, 0xC7, 0x80, 0x00, 0x79, 0xE0, 0x00, 0x1E, 0xF8, 0x00,
  0x07, 0xC0, 0xFF, 0xFC, 0x0F, 0xFF, 0xF0, 0xFF, 0xFF, 0x8F, 0x00, 0x7C,
  0xF0, 0x03, 0xCF, 0x00, 0x1E, 0xF0, 0x01, 0xEF, 0x00, 0x1E, 0xF0, 0x01,
  0xEF, 0x00, 0x1E, 0xF0, 0x03, 0xCF, 0x00, 0xF8, 0xFF, 0xFF, 0x0F, 0xFF,
  0xE0, 0xFF, 0xFF, 0x8F, 0x00, 0x7C, 0xF0, 0x01, 0xEF, 0x00, 0x1E, 0xF0,
  0x00, 0xEF, 0x00, 0x0F, 0xF0, 0x00, 0xFF, 0x00, 0x0F, 0xF0, 0x00, 0xEF,
  0x00, 0x1E, 0xF0, 0x03, 0xEF, 0x00, 0x7C, 0xFF, 0xFF, 0x8F, 0xFF, 0xF0,
  0xFF, 0xFC, 0x00, 0x00, 0xFE, 0x00, 0x0F, 0xFF, 0x00, 0x7F, 0xFE, 0x07,
  0xE0, 0x7C, 0x1E, 0x00, 0xF8, 0xF0, 0x01, 0xE3, 0xC0, 0x03, 0xDE, 0x00,
  0x0F, 0x78, 0x00, 0x3D, 0xC0, 0x00, 0xFF, 0x00, 0x00, 0x3C, 0x00, 0x00,
  0xF0, 0x00, 0x03, 0xC0, 0x00, 0x0F, 0x00, 0x00, 0x3C, 0x00, 0x00, 0xF0,
  0x00, 0x03, 0xC0, 0x00, 0x0F, 0x00, 0x00, 0x1C, 0x00, 0x00, 0x78, 0x00,
  0x3D, 0xE0, 0x00, 0xF3, 0xC0, 0x03, 0xCF, 0x00, 0x1E, 0x1E, 0x00, 0xF8,
  0x7E, 0x07, 0xC0, 0xFF, 0xFE, 0x00, 0xFF, 0xF0, 0x00, 0xFE, 0x00, 0xFF,
  0xF0, 0x07, 0xFF, 0xF0, 0x3F, 0xFF, 0xC1, 0xE0, 0x3F, 0x0F, 0x00, 0x7C,
  0x78, 0x00, 0xF3, 0xC0, 0x07, 0x9E, 0x00, 0x1E, 0xF0, 0x00, 0xF7, 0x80,
  0x03, 0xBC, 0x00, 0x1F, 0xE0, 0x00, 0xFF, 0x00, 0x07, 0xF8, 0x00, 0x3F,
  0xC0, 0x01, 0xFE, 0x00, 0x0F, 0xF0, 0x00, 0x7F, 0x80, 0x03, 0xFC, 0x00,
  0x1F, 0xE0, 0x00, 0xEF, 0x00, 0x0F, 0x78, 0x00, 0x7B, 0xC0, 0x07, 0x9E,
  0x00, 0x7C, 0xF0, 0x07, 0xC7, 0x80, 0xFC, 0x3F, 0xFF, 0xC1, 0xFF, 0xFC,
  0x0F, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0,
  0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00,
  0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xFF, 0xFF, 0x3F, 0xFF, 0xCF,
  0xFF, 0xF3, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0,
  0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00,
  0x3C, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F,
  0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0,
  0x00, 0xF0, 0x00, 0x3F, 0xFF, 0xCF, 0xFF, 0xF3, 0xFF, 0xFC, 0xF0, 0x00,
  0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F,
  0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0,
  0x00, 0xF0, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0xFF, 0x00, 0xFF, 0xFE,
  0x07, 0xE0, 0x7C, 0x1E, 0x00, 0xF8, 0xF0, 0x01, 0xE3, 0xC0, 0x03, 0xDE,
  0x00, 0x0F, 0x78, 0x00, 0x3D, 0xC0, 0x00, 0x07, 0x00, 0x00, 0x3C, 0x00,
  0x00, 0xF0, 0x00, 0x03, 0xC0, 0x00, 0x0F, 0x00, 0x00, 0x3C, 0x03, 0xFF,
  0xF0, 0x0F, 0xFF, 0xC0, 0x3F, 0xF7, 0x00, 0x01, 0xDE, 0x00, 0x07, 0x78,
  0x00, 0x1D, 0xE0, 0x00, 0x73, 0xC0, 0x01, 0xCF, 0x80, 0x07, 0x1F, 0x00,
  0x3C, 0x3E, 0x03, 0xF0, 0x7F, 0xFF, 0x80, 0xFF, 0xF8, 0x00, 0x7F, 0x80,
  0xF0, 0x00, 0x3F, 0xC0, 0x00, 0xFF, 0x00, 0x03, 0xFC, 0x00, 0x0F, 0xF0,
  0x00, 0x3F, 0xC0, 0x00, 0xFF, 0x00, 0x03, 0xFC, 0x00, 0x0F, 0xF0, 0x00,
  0x3F, 0xC0, 0x00, 0xFF, 0x00, 0x03, 0xFC, 0x00, 0x0F, 0xF0, 0x00, 0x3F,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0x3F, 0xC0,
  0x00, 0xFF, 0x00, 0x03, 0xFC, 0x00, 0x0F, 0xF0, 0x00, 0x3F, 0xC0, 0x00,
  0xFF, 0x00, 0x03, 0xFC, 0x00, 0x0F, 0xF0, 0x00, 0x3F, 0xC0, 0x00, 0xFF,
  0x00, 0x03, 0xFC, 0x00, 0x0F, 0xF0, 0x00, 0x3C, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0x00, 0x03, 0xC0, 0x00, 0xF0,
  0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00,
  0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03,
  0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0,
  0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xFC, 0x00, 0xFF, 0x00, 0x3F, 0xC0,
  0x1F, 0x78, 0x07, 0x9F, 0x07, 0xE3, 0xFF, 0xF0, 0x7F, 0xF0, 0x07, 0xF0,
  0x00, 0xF0, 0x00, 0xFB, 0xC0, 0x07, 0xCF, 0x00, 0x3E, 0x3C, 0x00, 0xF0,
  0xF0, 0x07, 0x83, 0xC0, 0x3C, 0x0F, 0x01, 0xF0, 0x3C, 0x0F, 0x80, 0xF0,
  0x7C, 0x03, 0xC3, 0xE0, 0x0F, 0x1F, 0x00, 0x3C, 0x78, 0x00, 0xF3, 0xC0,
  0x03, 0xDF, 0x80, 0x0F, 0xFE, 0x00, 0x3F, 0xFC, 0x00, 0xFE, 0xF8, 0x03,
  0xF1, 0xF0, 0x0F, 0x83, 0xC0, 0x3C, 0x07, 0x80, 0xF0, 0x1F, 0x03, 0xC0,
  0x3E, 0x0F, 0x00, 0x78, 0x3C, 0x01, 0xF0, 0xF0, 0x03, 0xE3, 0xC0, 0x07,
  0x8F, 0x00, 0x0F, 0x3C, 0x00, 0x3E, 0xF0, 0x00, 0x7C, 0xF0, 0x00, 0x3C,
  0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00,
  0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00,
  0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C,
  0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00,
  0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xC0, 0xF8, 0x00, 0x00, 0xFF, 0xE0, 0x00, 0x0F, 0xFF, 0x00,
  0x00, 0x7F, 0xFC, 0x00, 0x07, 0xFF, 0xE0, 0x00, 0x3F, 0xFF, 0x00, 0x01,
  0xFF, 0xFC, 0x00, 0x1F, 0xFF, 0xE0, 0x00, 0xEF, 0xF7, 0x00, 0x0F, 0x7F,
  0xBC, 0x00, 0x7B, 0xFC, 0xE0, 0x03, 0x9F, 0xE7, 0x80, 0x3C, 0xFF, 0x3C,
  0x01, 0xC7, 0xF8, 0xE0, 0x1E, 0x3F, 0xC7, 0x80, 0xF1, 0xFE, 0x1C, 0x07,
  0x0F, 0xF0, 0xF0, 0x78, 0x7F, 0x87, 0x83, 0x83, 0xFC, 0x1C, 0x3C, 0x1F,
  0xE0, 0xF1, 0xE0, 0xFF, 0x03, 0x8E, 0x07, 0xF8, 0x1E, 0xF0, 0x3F, 0xC0,
  0xF7, 0x01, 0xFE, 0x03, 0xF8, 0x0F, 0xF0, 0x1F, 0xC0, 0x7F, 0x80, 0x7C,
  0x03, 0xFC, 0x03, 0xE0, 0x1F, 0xE0, 0x1E, 0x00, 0xFF, 0x00, 0x70, 0x07,
  0x80, 0xF0, 0x00, 0x3F, 0xC0, 0x00, 0xFF, 0x80, 0x03, 0xFF, 0x00, 0x0F,
  0xFC, 0x00, 0x3F, 0xF8, 0x00, 0xFF, 0xF0, 0x03, 0xFB, 0xC0, 0x0F, 0xE7,
  0x80, 0x3F, 0x9E, 0x00, 0xFE, 0x3C, 0x03, 0xF8, 0x78, 0x0F, 0xE1, 0xE0,
  0x3F, 0x83, 0xC0, 0xFE, 0x0F, 0x83, 0xF8, 0x1E, 0x0F, 0xE0, 0x3C, 0x3F,
  0x80, 0xF8, 0xFE, 0x01, 0xE3, 0xF8, 0x03, 0xCF, 0xE0, 0x0F, 0xBF, 0x80,
  0x1E, 0xFE, 0x00, 0x3F, 0xF8, 0x00, 0xFF, 0xE0, 0x01, 0xFF, 0x80, 0x03,
  0xFE, 0x00, 0x0F, 0xF8, 0x00, 0x1F, 0xE0, 0x00, 0x7C, 0x00, 0xFE, 0x00,
  0x07, 0xFF, 0x80, 0x1F, 0xFF, 0x80, 0x7C, 0x0F, 0x81, 0xE0, 0x0F, 0x87,
  0x80, 0x0F, 0x0F, 0x00, 0x0F, 0x3C, 0x00, 0x1E, 0x78, 0x00, 0x1E, 0xE0,
  0x00, 0x3D, 0xC0, 0x00, 0x7F, 0x80, 0x00, 0xFF, 0x00, 0x01, 0xFE, 0x00,
  0x03, 0xFC, 0x00, 0x07, 0xF8, 0x00, 0x0F, 0xF0, 0x00, 0x1F, 0xE0, 0x00,
  0x3D, 0xC0, 0x00, 0x7B, 0x80, 0x00, 0xF7, 0x80, 0x01, 0xEF, 0x00, 0x07,
  0x8F, 0x00, 0x0F, 0x1E, 0x00, 0x3C, 0x1E, 0x00, 0xF8, 0x1F, 0x03, 0xE0,
  0x1F, 0xFF, 0x80, 0x1F, 0xFE, 0x00, 0x0F, 0xE0, 0x00, 0xFF, 0xFC, 0x07,
  0xFF, 0xFC, 0x3F, 0xFF, 0xF1, 0xE0, 0x0F, 0xCF, 0x00, 0x1F, 0x78, 0x00,
  0x7B, 0xC0, 0x03, 0xDE, 0x00, 0x0F, 0xF0, 0x00, 0x7F, 0x80, 0x03, 0xFC,
  0x00, 0x1F, 0xE0, 0x01, 0xEF, 0x00, 0x0F, 0x78, 0x00, 0xFB, 0xC0, 0x0F,
  0x9F, 0xFF, 0xF8, 0xFF, 0xFF, 0x87, 0xFF, 0xE0, 0x3C, 0x00, 0x01, 0xE0,
  0x00, 0x0F, 0x00, 0x00, 0x78, 0x00, 0x03, 0xC0, 0x00, 0x1E, 0x00, 0x00,
  0xF0, 0x00, 0x07, 0x80, 0x00, 0x3C, 0x00, 0x01, 0xE0, 0x00, 0x0F, 0x00,
  0x00, 0x00, 0x00, 0xFE, 0x00, 0x07, 0xFF, 0x80, 0x1F, 0xFF, 0x80, 0x7C,
  0x0F, 0x81, 0xE0, 0x0F, 0x87, 0x80, 0x0F, 0x0F, 0x00, 0x0F, 0x3C, 0x00,
  0x1E, 0x78, 0x00, 0x1E, 0xF0, 0x00, 0x3D, 0xC0, 0x00, 0x7F, 0x80, 0x00,
  0xFF, 0x00, 0x01, 0xFE, 0x00, 0x03, 0xFC, 0x00, 0x07, 0xF8, 0x00, 0x0F,
  0xF0, 0x00, 0x1F, 0xE0, 0x00, 0x3D, 0xC0, 0x00, 0x7B, 0xC0, 0x00, 0xF7,
  0x80, 0x01, 0xEF, 0x00, 0x07, 0x8F, 0x00, 0x0F, 0x1E, 0x00, 0x3C, 0x1E,
  0x00, 0xF8, 0x1F, 0x03, 0xE0, 0x1F, 0xFF, 0x80, 0x1F, 0xFF, 0x00, 0x0F,
  0xFF, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x1F, 0x00, 0x00,
  0x0C, 0xFF, 0xFC, 0x07, 0xFF, 0xF8, 0x3F, 0xFF, 0xF1, 0xE0, 0x0F, 0x8F,
  0x00, 0x1E, 0x78, 0x00, 0xF3, 0xC0, 0x03, 0xDE, 0x00, 0x1E, 0xF0, 0x00,
  0xF7, 0x80, 0x07, 0xBC, 0x00, 0x3D, 0xE0, 0x03, 0xCF, 0x00, 0x3E, 0x78,
  0x03, 0xE3, 0xFF, 0xFE, 0x1F, 0xFF, 0xE0, 0xFF, 0xFE, 0x07, 0x80, 0xF0,
  0x3C, 0x03, 0xC1, 0xE0, 0x1E, 0x0F, 0x00, 0x78, 0x78, 0x03, 0xC3, 0xC0,
  0x0F, 0x1E, 0x00, 0x78, 0xF0, 0x01, 0xE7, 0x80, 0x0F, 0x3C, 0x00, 0x3D,
  0xE0, 0x01, 0xEF, 0x00, 0x07, 0x80, 0x01, 0xFC, 0x00, 0xFF, 0xF0, 0x1F,
  0xFF, 0xC3, 0xE0, 0x7C, 0x7C, 0x01, 0xE7, 0x80, 0x0F, 0x78, 0x00, 0xF7,
  0x00, 0x0F, 0x70, 0x00, 0x77, 0x80, 0x00, 0x78, 0x00, 0x03, 0xE0, 0x00,
  0x1F, 0x80, 0x00, 0xFF, 0x00, 0x03, 0xFE, 0x00, 0x0F, 0xF8, 0x00, 0x1F,
  0xC0, 0x00, 0x3E, 0x00, 0x01, 0xF0, 0x00, 0x0F, 0xE0, 0x00, 0xFF, 0x00,
  0x07, 0xF0, 0x00, 0xFF, 0x00, 0x0F, 0x78, 0x01, 0xF3, 0xE0, 0x3E, 0x1F,
  0xFF, 0xC0, 0xFF, 0xF8, 0x01, 0xFE, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xC0, 0x1E, 0x00, 0x00, 0x78, 0x00, 0x01, 0xE0, 0x00,
  0x07, 0x80, 0x00, 0x1E, 0x00, 0x00, 0x78, 0x00, 0x01, 0xE0, 0x00, 0x07,
  0x80, 0x00, 0x1E, 0x00, 0x00, 0x78, 0x00, 0x01, 0xE0, 0x00, 0x07, 0x80,
  0x00, 0x1E, 0x00, 0x00, 0x78, 0x00, 0x01, 0xE0, 0x00, 0x07, 0x80, 0x00,
  0x1E, 0x00, 0x00, 0x78, 0x00, 0x01, 0xE0, 0x00, 0x07, 0x80, 0x00, 0x1E,
  0x00, 0x00, 0x78, 0x00, 0x01, 0xE0, 0x00, 0x07, 0x80, 0x00, 0x1E, 0x00,
  0x00, 0x78, 0x00, 0xE0, 0x00, 0x7E, 0x00, 0x07, 0xE0, 0x00, 0x7E, 0x00,
  0x07, 0xE0, 0x00, 0x7E, 0x00, 0x07, 0xE0, 0x00, 0x7E, 0x00, 0x07, 0xE0,
  0x00, 0x7E, 0x00, 0x07, 0xE0, 0x00, 0x7E, 0x00, 0x07, 0xE0, 0x00, 0x7E,
  0x00, 0x07, 0xE0, 0x00, 0x7E, 0x00, 0x07, 0xE0, 0x00, 0x7E, 0x00, 0x07,
  0xE0, 0x00, 0x7E, 0x00, 0x07, 0xF0, 0x00, 0xFF, 0x00, 0x0F, 0xF0, 0x00,
  0xFF, 0x80, 0x0F, 0x78, 0x01, 0xE3, 0xE0, 0x7C, 0x1F, 0xFF, 0x80, 0xFF,
  0xF0, 0x03, 0xFC, 0x00, 0xF8, 0x00, 0x0F, 0xBC, 0x00, 0x07, 0x9E, 0x00,
  0x03, 0xC7, 0x80, 0x03, 0xC3, 0xC0, 0x01, 0xE1, 0xE0, 0x00, 0xF0, 0x78,
  0x00, 0xF0, 0x3C, 0x00, 0x78, 0x1E, 0x00, 0x3C, 0x07, 0x80, 0x3C, 0x03,
  0xC0, 0x1E, 0x01, 0xE0, 0x0F, 0x00, 0x78, 0x0F, 0x00, 0x3C, 0x07, 0x80,
  0x0E, 0x03, 0x80, 0x07, 0x83, 0xC0, 0x03, 0xC1, 0xE0, 0x00, 0xE0, 0xE0,
  0x00, 0x78, 0xF0, 0x00, 0x3C, 0x78, 0x00, 0x0E, 0x38, 0x00, 0x07, 0xBC,
  0x00, 0x03, 0xDE, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x1F,
  0x00, 0x00, 0x0F, 0x80, 0x00, 0x07, 0xC0, 0x00, 0x01, 0xC0, 0x00, 0xF0,
  0x01, 0xE0, 0x03, 0xDC, 0x00, 0x78, 0x00, 0xE7, 0x80, 0x1E, 0x00, 0x79,
  0xE0, 0x0F, 0xC0, 0x1E, 0x78, 0x03, 0xF0, 0x07, 0x9E, 0x00, 0xFC, 0x01,
  0xC3, 0x80, 0x3F, 0x00, 0xF0, 0xF0, 0x1F, 0xE0, 0x3C, 0x3C, 0x07, 0x38,
  0x0F, 0x0F, 0x01, 0xCE, 0x03, 0x81, 0xC0, 0xF3, 0x80, 0xE0, 0x78, 0x3C,
  0xF0, 0x78, 0x1E, 0x0E, 0x1C, 0x1E, 0x07, 0x83, 0x87, 0x07, 0x00, 0xE1,
  0xE1, 0xE1, 0xC0, 0x38, 0x70, 0x38, 0xF0, 0x0F, 0x1C, 0x0E, 0x3C, 0x03,
  0xC7, 0x03, 0x8E, 0x00, 0x73, 0xC0, 0xF3, 0x80, 0x1C, 0xE0, 0x1C, 0xE0,
  0x07, 0xB8, 0x07, 0x78, 0x01, 0xFE, 0x01, 0xDE, 0x00, 0x3F, 0x00, 0x3F,
  0x00, 0x0F, 0xC0, 0x0F, 0xC0, 0x03, 0xF0, 0x03, 0xF0, 0x00, 0xFC, 0x00,
  0xFC, 0x00, 0x3E, 0x00, 0x1E, 0x00, 0x07, 0x80, 0x07, 0x80, 0x01, 0xE0,
  0x01, 0xE0, 0x00, 0xF8, 0x00, 0x3E, 0xF8, 0x00, 0xF8, 0xF0, 0x01, 0xE0,
  0xF0, 0x07, 0xC1, 0xF0, 0x0F, 0x01, 0xE0, 0x3C, 0x03, 0xE0, 0xF8, 0x03,
  0xC1, 0xE0, 0x03, 0xC7, 0xC0, 0x07, 0xCF, 0x00, 0x07, 0xBC, 0x00, 0x07,
  0xF8, 0x00, 0x0F, 0xE0, 0x00, 0x0F, 0x80, 0x00, 0x1F, 0x00, 0x00, 0x3F,
  0x00, 0x00, 0xFE, 0x00, 0x03, 0xFE, 0x00, 0x07, 0xBC, 0x00, 0x1E, 0x3C,
  0x00, 0x7C, 0x7C, 0x00, 0xF0, 0x78, 0x03, 0xE0, 0x78, 0x07, 0x80, 0xF8,
  0x1E, 0x00, 0xF0, 0x7C, 0x01, 0xF0, 0xF0, 0x01, 0xE3, 0xC0, 0x01, 0xEF,
  0x80, 0x03, 0xE0, 0xF8, 0x00, 0x1F, 0x78, 0x00, 0x1E, 0x3C, 0x00, 0x3C,
  0x3C, 0x00, 0x3C, 0x1E, 0x00, 0x78, 0x1E, 0x00, 0x78, 0x0F, 0x00, 0xF0,
  0x0F, 0x00, 0xF0, 0x07, 0x81, 0xE0, 0x07, 0x81, 0xE0, 0x03, 0xC3, 0xC0,
  0x03, 0xC3, 0x80, 0x01, 0xE7, 0x80, 0x00, 0xEF, 0x00, 0x00, 0xFF, 0x00,
  0x00, 0x7E, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x3C, 0x00,
  0x00, 0x3C, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x3C, 0x00,
  0x00, 0x3C, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x3C, 0x00,
  0x00, 0x3C, 0x00, 0x00, 0x3C, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xF0, 0x00, 0x1E, 0x00, 0x03, 0xE0, 0x00, 0x3C, 0x00, 0x07, 0x80,
  0x00, 0xF8, 0x00, 0x0F, 0x00, 0x01, 0xE0, 0x00, 0x3E, 0x00, 0x03, 0xC0,
  0x00, 0x78, 0x00, 0x0F, 0x80, 0x00, 0xF0, 0x00, 0x1E, 0x00, 0x03, 0xC0,
  0x00, 0x3C, 0x00, 0x07, 0x80, 0x00, 0xF0, 0x00, 0x0F, 0x00, 0x01, 0xE0,
  0x00, 0x3C, 0x00, 0x03, 0xC0, 0x00, 0x78, 0x00, 0x0F, 0x00, 0x00, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xFF, 0xFF, 0xFF, 0x0E, 0x1C,
  0x38, 0x70, 0xE1, 0xC3, 0x87, 0x0E, 0x1C, 0x38, 0x70, 0xE1, 0xC3, 0x87,
  0x0E, 0x1C, 0x38, 0x70, 0xE1, 0xC3, 0x87, 0x0E, 0x1C, 0x38, 0x70, 0xE1,
  0xC3, 0x87, 0x0F, 0xFF, 0xFF, 0x80, 0xE0, 0x01, 0xE0, 0x01, 0xC0, 0x03,
  0xC0, 0x03, 0x80, 0x07, 0x00, 0x0F, 0x00, 0x0E, 0x00, 0x1C, 0x00, 0x3C,
  0x00, 0x38, 0x00, 0x78, 0x00, 0x70, 0x00, 0xE0, 0x01, 0xE0, 0x01, 0xC0,
  0x03, 0xC0, 0x07, 0x80, 0x07, 0x00, 0x0F, 0x00, 0x0E, 0x00, 0x1C, 0x00,
  0x3C, 0x00, 0x38, 0x00, 0x78, 0x00, 0x70, 0x00, 0xE0, 0x01, 0xE0, 0x01,
  0xC0, 0x03, 0xC0, 0x07, 0x80, 0xFF, 0xFF, 0xFF, 0x0F, 0x0F, 0x0F, 0x0F,
  0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
  0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
  0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0xFF, 0xFF, 0xFF, 0x03, 0x80, 0x07, 0x00,
  0x1F, 0x00, 0x3E, 0x00, 0xFE, 0x01, 0xDC, 0x07, 0xB8, 0x0E, 0x38, 0x1C,
  0x70, 0x78, 0xF0, 0xE0, 0xE3, 0xC1, 0xC7, 0x01, 0xDE, 0x03, 0xC0, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xF8, 0x7C, 0x3C, 0x1E, 0x0F, 0x07,
  0x03, 0xF0, 0x03, 0xFF, 0x01, 0xFF, 0xE0, 0xF0, 0x7C, 0x78, 0x0F, 0x9C,
  0x01, 0xE0, 0x00, 0x78, 0x00, 0x1E, 0x00, 0x07, 0x80, 0xFF, 0xE1, 0xFF,
  0xF8, 0xFF, 0xFE, 0x7C, 0x07, 0x9E, 0x01, 0xEF, 0x00, 0x7B, 0xC0, 0x1E,
  0xF0, 0x07, 0xBC, 0x03, 0xE7, 0xC3, 0xF8, 0xFF, 0xFE, 0x1F, 0xF3, 0x83,
  0xF0, 0xF0, 0xE0, 0x00, 0x70, 0x00, 0x38, 0x00, 0x1C, 0x00, 0x0E, 0x00,
  0x07, 0x00, 0x03, 0x80, 0x01, 0xC0, 0x00, 0xE3, 0xF0, 0x77, 0xFE, 0x3F,
  0xFF, 0x9F, 0x83, 0xEF, 0x00, 0xF7, 0x80, 0x3F, 0x80, 0x1F, 0xC0, 0x0F,
  0xE0, 0x03, 0xF0, 0x01, 0xF8, 0x00, 0xFC, 0x00, 0x7E, 0x00, 0x3F, 0x00,
  0x1F, 0x80, 0x1F, 0xC0, 0x0F, 0xF0, 0x07, 0xF8, 0x07, 0xBF, 0x07, 0xDF,
  0xFF, 0xCE, 0xFF, 0xC7, 0x1F, 0x80, 0x03, 0xF0, 0x03, 0xFF, 0x01, 0xFF,
  0xF0, 0xF8, 0x7C, 0x78, 0x07, 0x9E, 0x00, 0xEF, 0x00, 0x3F, 0xC0, 0x0F,
  0xF0, 0x00, 0x38, 0x00, 0x0E, 0x00, 0x03, 0x80, 0x00, 0xE0, 0x00, 0x3C,
  0x00, 0x0F, 0x00, 0x03, 0xC0, 0x0F, 0x78, 0x03, 0x9E, 0x01, 0xE3, 0xE0,
  0xF0, 0x7F, 0xF8, 0x0F, 0xFC, 0x00, 0xFC, 0x00, 0x00, 0x03, 0xC0, 0x00,
  0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0, 0x00, 0x3C,
  0x00, 0x0F, 0x03, 0xE3, 0xC3, 0xFE, 0xF1, 0xFF, 0xFC, 0xF8, 0x3F, 0x7C,
  0x07, 0xDE, 0x00, 0xF7, 0x00, 0x3F, 0xC0, 0x0F, 0xF0, 0x03, 0xFC, 0x00,
  0xFF, 0x00, 0x3F, 0xC0, 0x0F, 0xF0, 0x03, 0xFC, 0x00, 0xFF, 0x00, 0x3D,
  0xC0, 0x0F, 0x78, 0x03, 0xDF, 0x01, 0xF3, 0xE0, 0xFC, 0x7F, 0xFF, 0x0F,
  0xFB, 0xC0, 0xF8, 0x70, 0x03, 0xF0, 0x03, 0xFF, 0x01, 0xFF, 0xE0, 0xF8,
  0x7C, 0x78, 0x07, 0x1E, 0x01, 0xEF, 0x00, 0x3B, 0xC0, 0x0E, 0xF0, 0x03,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0x00, 0x38, 0x00, 0x0F,
  0x00, 0x03, 0xC0, 0x00, 0x78, 0x01, 0x1F, 0x00, 0xE3, 0xE0, 0xF8, 0x7F,
  0xFC, 0x0F, 0xFE, 0x00, 0xFE, 0x00, 0x01, 0xF8, 0x1F, 0xC1, 0xFE, 0x1F,
  0x00, 0xF0, 0x07, 0x00, 0x38, 0x01, 0xC0, 0xFF, 0xF7, 0xFF, 0xBF, 0xFC,
  0x1C, 0x00, 0xE0, 0x07, 0x00, 0x38, 0x01, 0xC0, 0x0E, 0x00, 0x70, 0x03,
  0x80, 0x1C, 0x00, 0xE0, 0x07, 0x00, 0x38, 0x01, 0xC0, 0x0E, 0x00, 0x70,
  0x03, 0x80, 0x1C, 0x00, 0xE0, 0x07, 0x00, 0x07, 0xE3, 0xC7, 0xFE, 0xF3,
  0xFF, 0xFD, 0xF8, 0x7F, 0x78, 0x07, 0xFE, 0x00, 0xFF, 0x00, 0x3F, 0xC0,
  0x0F, 0xE0, 0x03, 0xF8, 0x00, 0xFE, 0x00, 0x3F, 0x80, 0x0F, 0xE0, 0x03,
  0xF8, 0x00, 0xFF, 0x00, 0x3F, 0xC0, 0x0F, 0x70, 0x07, 0xDE, 0x01, 0xF3,
  0xE1, 0xFC, 0xFF, 0xFF, 0x1F, 0xFB, 0xC1, 0xF8, 0xE0, 0x00, 0x38, 0x00,
  0x0E, 0x20, 0x07, 0x9C, 0x03, 0xE7, 0xC1, 0xF0, 0xFF, 0xF8, 0x1F, 0xFC,
  0x01, 0xFC, 0x00, 0xE0, 0x00, 0xE0, 0x00, 0xE0, 0x00, 0xE0, 0x00, 0xE0,
  0x00, 0xE0, 0x00, 0xE0, 0x00, 0xE0, 0x00, 0xE1, 0xF8, 0xE7, 0xFC, 0xEF,
  0xFE, 0xFC, 0x1F, 0xF8, 0x0F, 0xF0, 0x0F, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
  0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
  0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
  0x07, 0xE0, 0x07, 0x6F, 0xF7, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0E, 0x0F, 0x0F, 0x0E, 0x00, 0x00,
  0x00, 0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
  0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
  0x0F, 0x0F, 0x0F, 0x0F, 0x1E, 0xFE, 0xFC, 0xF8, 0xE0, 0x00, 0x70, 0x00,
  0x38, 0x00, 0x1C, 0x00, 0x0E, 0x00, 0x07, 0x00, 0x03, 0x80, 0x01, 0xC0,
  0x00, 0xE0, 0x1F, 0x70, 0x0F, 0x38, 0x0F, 0x1C, 0x0F, 0x0E, 0x0F, 0x07,
  0x0F, 0x03, 0x8F, 0x01, 0xCF, 0x80, 0xEF, 0x80, 0x7F, 0x80, 0x3F, 0xE0,
  0x1F, 0xF8, 0x0F, 0xBC, 0x07, 0x8F, 0x03, 0x83, 0xC1, 0xC1, 0xF0, 0xE0,
  0x78, 0x70, 0x1E, 0x38, 0x0F, 0x9C, 0x03, 0xCE, 0x00, 0xF7, 0x00, 0x3C,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xE3, 0xF0, 0x1F, 0x87, 0x3F, 0xE3, 0xFF, 0x3F, 0xFF,
  0xBF, 0xFD, 0xF8, 0x3F, 0xC1, 0xFF, 0x00, 0xFC, 0x07, 0xF8, 0x07, 0xC0,
  0x3F, 0x80, 0x1E, 0x00, 0xFC, 0x00, 0xE0, 0x07, 0xE0, 0x07, 0x00, 0x3F,
  0x00, 0x38, 0x01, 0xF8, 0x01, 0xC0, 0x0F, 0xC0, 0x0E, 0x00, 0x7E, 0x00,
  0x70, 0x03, 0xF0, 0x03, 0x80, 0x1F, 0x80, 0x1C, 0x00, 0xFC, 0x00, 0xE0,
  0x07, 0xE0, 0x07, 0x00, 0x3F, 0x00, 0x38, 0x01, 0xF8, 0x01, 0xC0, 0x0F,
  0xC0, 0x0E, 0x00, 0x7E, 0x00, 0x70, 0x03, 0xF0, 0x03, 0x80, 0x1C, 0xE1,
  0xF8, 0xE7, 0xFC, 0xEF, 0xFE, 0xFC, 0x1F, 0xF8, 0x0F, 0xF0, 0x0F, 0xE0,
  0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
  0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0,
  0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0x03, 0xF8, 0x01, 0xFF, 0xC0,
  0x7F, 0xFC, 0x1F, 0x07, 0xC7, 0xC0, 0x7C, 0xF0, 0x07, 0x9C, 0x00, 0x77,
  0x80, 0x0F, 0xF0, 0x01, 0xFE, 0x00, 0x3F, 0xC0, 0x07, 0xF8, 0x00, 0xFF,
  0x00, 0x1F, 0xE0, 0x03, 0xFC, 0x00, 0x7B, 0x80, 0x0E, 0x78, 0x03, 0xCF,
  0x80, 0xF8, 0xF8, 0x3E, 0x0F, 0xFF, 0x80, 0xFF, 0xE0, 0x07, 0xF0, 0x00,
  0xE3, 0xF0, 0x77, 0xFE, 0x3F, 0xFF, 0x9F, 0x87, 0xEF, 0x00, 0xF7, 0x00,
  0x7F, 0x80, 0x1F, 0xC0, 0x0F, 0xE0, 0x03, 0xF0, 0x01, 0xF8, 0x00, 0xFC,
  0x00, 0x7E, 0x00, 0x3F, 0x00, 0x1F, 0x80, 0x1F, 0xC0, 0x0F, 0xE0, 0x0F,
  0xF8, 0x07, 0xBF, 0x0F, 0xDF, 0xFF, 0xCE, 0xFF, 0xC7, 0x1F, 0x83, 0x80,
  0x01, 0xC0, 0x00, 0xE0, 0x00, 0x70, 0x00, 0x38, 0x00, 0x1C, 0x00, 0x0E,
  0x00, 0x07, 0x00, 0x00, 0x07, 0xE3, 0xC3, 0xFE, 0xF1, 0xFF, 0xFC, 0xF8,
  0x3F, 0x7C, 0x07, 0xDE, 0x00, 0xFF, 0x00, 0x3F, 0xC0, 0x0F, 0xF0, 0x03,
  0xFC, 0x00, 0xFF, 0x00, 0x3F, 0xC0, 0x0F, 0xF0, 0x03, 0xFC, 0x00, 0xFF,
  0x00, 0x3D, 0xC0, 0x0F, 0x78, 0x03, 0xDF, 0x01, 0xF3, 0xE0, 0xFC, 0x7F,
  0xFF, 0x0F, 0xFB, 0xC1, 0xF8, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03,
  0xC0, 0x00, 0xF0, 0x00, 0x3C, 0x00, 0x0F, 0x00, 0x03, 0xC0, 0x00, 0xF0,
  0xE3, 0xFD, 0xFF, 0xFF, 0xF0, 0xF8, 0x3C, 0x0F, 0x03, 0xC0, 0xF0, 0x3C,
  0x0F, 0x03, 0xC0, 0xF0, 0x3C, 0x0F, 0x03, 0xC0, 0xF0, 0x3C, 0x0F, 0x03,
  0xC0, 0xF0, 0x3C, 0x00, 0x07, 0xF0, 0x0F, 0xFE, 0x0F, 0xFF, 0x8F, 0x83,
  0xE7, 0x80, 0xF3, 0x80, 0x3D, 0xC0, 0x1E, 0xF0, 0x00, 0x7C, 0x00, 0x1F,
  0xE0, 0x07, 0xFE, 0x00, 0xFF, 0xC0, 0x07, 0xF0, 0x00, 0x78, 0x00, 0x1F,
  0xE0, 0x0F, 0xF0, 0x07, 0xBC, 0x03, 0xDF, 0x07, 0xC7, 0xFF, 0xC1, 0xFF,
  0xC0, 0x3F, 0x80, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF0, 0x0F, 0x0F, 0xFF,
  0xFF, 0xFF, 0xFF, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF0,
  0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0xF0,
  0x0F, 0x00, 0xF0, 0x07, 0xF0, 0x7F, 0x03, 0xF0, 0xE0, 0x07, 0xE0, 0x07,
  0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07,
  0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07, 0xE0, 0x07,
  0xE0, 0x07, 0xE0, 0x07, 0xF0, 0x0F, 0xF0, 0x0F, 0xF8, 0x3F, 0x7F, 0xFF,
  0x3F, 0xF7, 0x0F, 0xC7, 0xF0, 0x01, 0xDE, 0x00, 0xF7, 0x80, 0x3C, 0xE0,
  0x0E, 0x3C, 0x07, 0x8F, 0x01, 0xE1, 0xC0, 0x70, 0x78, 0x3C, 0x1E, 0x0F,
  0x03, 0x83, 0x80, 0xE1, 0xE0, 0x3C, 0x78, 0x07, 0x1C, 0x01, 0xC7, 0x00,
  0x3B, 0x80, 0x0E, 0xE0, 0x03, 0xB8, 0x00, 0x7C, 0x00, 0x1F, 0x00, 0x07,
  0xC0, 0x00, 0xE0, 0x00, 0x38, 0x00, 0xF0, 0x0F, 0x00, 0xFF, 0x00, 0xF0,
  0x0F, 0x70, 0x0F, 0x00, 0xE7, 0x01, 0xF0, 0x0E, 0x78, 0x1F, 0x81, 0xE7,
  0x81, 0xF8, 0x1E, 0x38, 0x1F, 0x81, 0xC3, 0x83, 0x9C, 0x1C, 0x3C, 0x39,
  0xC3, 0xC3, 0xC3, 0x9C, 0x38, 0x1C, 0x71, 0xE3, 0x81, 0xC7, 0x0E, 0x38,
  0x1E, 0x70, 0xE7, 0x80, 0xEE, 0x0E, 0x70, 0x0E, 0xE0, 0x77, 0x00, 0xEE,
  0x07, 0x70, 0x0F, 0xE0, 0x7E, 0x00, 0x7C, 0x03, 0xE0, 0x07, 0xC0, 0x3E,
  0x00, 0x7C, 0x03, 0xE0, 0x03, 0x80, 0x1C, 0x00, 0x38, 0x01, 0xC0, 0xF0,
  0x07, 0xDE, 0x01, 0xE3, 0xC0, 0xF0, 0xF0, 0x7C, 0x1E, 0x1E, 0x03, 0x8F,
  0x00, 0xF3, 0xC0, 0x1F, 0xE0, 0x07, 0xF0, 0x00, 0xFC, 0x00, 0x1E, 0x00,
  0x07, 0x80, 0x03, 0xF0, 0x01, 0xFE, 0x00, 0x77, 0x80, 0x3C, 0xF0, 0x1E,
  0x1C, 0x07, 0x87, 0x83, 0xC0, 0xF1, 0xE0, 0x3C, 0x78, 0x07, 0xBC, 0x01,
  0xF0, 0xF8, 0x03, 0xDE, 0x00, 0xF7, 0x80, 0x3C, 0xF0, 0x0E, 0x3C, 0x07,
  0x8F, 0x01, 0xE1, 0xE0, 0x70, 0x78, 0x3C, 0x0E, 0x0F, 0x03, 0xC3, 0x80,
  0xF1, 0xE0, 0x1C, 0x78, 0x07, 0x9C, 0x01, 0xEF, 0x00, 0x3B, 0x80, 0x0F,
  0xE0, 0x03, 0xF8, 0x00, 0x7C, 0x00, 0x1F, 0x00, 0x07, 0xC0, 0x00, 0xE0,
  0x00, 0x38, 0x00, 0x1E, 0x00, 0x07, 0x00, 0x01, 0xC0, 0x00, 0xF0, 0x00,
  0x78, 0x00, 0xFC, 0x00, 0x3F, 0x00, 0x0F, 0x00, 0x00, 0xFF, 0xFF, 0x7F,
  0xFF, 0xBF, 0xFF, 0xC0, 0x03, 0xC0, 0x03, 0xE0, 0x01, 0xE0, 0x01, 0xE0,
  0x01, 0xE0, 0x01, 0xF0, 0x00, 0xF0, 0x00, 0xF0, 0x00, 0xF8, 0x00, 0x78,
  0x00, 0x78, 0x00, 0x78, 0x00, 0x7C, 0x00, 0x3C, 0x00, 0x3C, 0x00, 0x3E,
  0x00, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x30, 0x0F, 0x01,
  0xF0, 0x3C, 0x03, 0x80, 0x78, 0x07, 0x00, 0x70, 0x07, 0x00, 0x70, 0x07,
  0x00, 0x70, 0x07, 0x00, 0x70, 0x0F, 0x00, 0xF0, 0x0F, 0x01, 0xE0, 0xFC,
  0x0F, 0x80, 0xFC, 0x01, 0xE0, 0x0F, 0x00, 0xF0, 0x0F, 0x00, 0x70, 0x07,
  0x00, 0x70, 0x07, 0x00, 0x70, 0x07, 0x00, 0x70, 0x07, 0x80, 0x78, 0x03,
  0xC0, 0x1E, 0x00, 0xF0, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xC0, 0x0F, 0x00, 0xF8, 0x03, 0xC0,
  0x1C, 0x01, 0xE0, 0x1E, 0x00, 0xE0, 0x0E, 0x00, 0xE0, 0x0E, 0x00, 0xE0,
  0x0E, 0x00, 0xE0, 0x0E, 0x00, 0xF0, 0x0F, 0x00, 0x78, 0x03, 0xF0, 0x1F,
  0x03, 0xF0, 0x78, 0x0F, 0x00, 0xF0, 0x0E, 0x00, 0xE0, 0x0E, 0x00, 0xE0,
  0x0E, 0x00, 0xE0, 0x0E, 0x01, 0xE0, 0x1E, 0x01, 0xE0, 0x3C, 0x0F, 0x80,
  0xF0, 0x0C, 0x00, 0x0F, 0x80, 0x0E, 0x3F, 0xC0, 0x3C, 0xFF, 0xC0, 0x73,
  0xC7, 0xC0, 0xE7, 0x03, 0xE3, 0xCE, 0x03, 0xFF, 0x3C, 0x03, 0xFC, 0x00,
  0x01, 0xF0 };

const GFXglyph roboto36pt7bGlyphs[] = {
  {     0,   1,   1,  10,    0,    0 },   // 0x20 ' '
  {     1,   4,  29,  10,    3,  -28 },   // 0x21 '!'
  {    16,   8,  10,  13,    3,  -29 },   // 0x22 '"'
  {    26,  22,  29,  25,    2,  -28 },   // 0x23 '#'
  {   106,  18,  37,  22,    2,  -32 },   // 0x24 '$'
  {   190,  26,  29,  29,    2,  -28 },   // 0x25 '%'
  {   285,  23,  29,  25,    2,  -28 },   // 0x26 '&'
  {   369,   3,   9,   7,    2,  -29 },   // 0x27 '''
  {   373,  10,  41,  14,    3,  -31 },   // 0x28 '('
  {   425,  10,  41,  14,    1,  -31 },   // 0x29 ')'
  {   477,  16,  17,  17,    1,  -28 },   // 0x2A '*'
  {   511,  19,  20,  23,    2,  -22 },   // 0x2B '+'
  {   559,   5,  10,   8,    1,   -3 },   // 0x2C ','
  {   566,   9,   3,  11,    1,  -13 },   // 0x2D '-'
  {   570,   4,   4,  11,    3,   -3 },   // 0x2E '.'
  {   572,  15,  31,  16,    0,  -28 },   // 0x2F '/'
  {   631,  18,  29,  22,    2,  -28 },   // 0x30 '0'
  {   697,  11,  29,  22,    3,  -28 },   // 0x31 '1'
  {   737,  19,  29,  22,    2,  -28 },   // 0x32 '2'
  {   806,  18,  29,  22,    2,  -28 },   // 0x33 '3'
  {   872,  21,  29,  22,    1,  -28 },   // 0x34 '4'
  {   949,  18,  29,  22,    3,  -28 },   // 0x35 '5'
  {  1015,  18,  29,  22,    3,  -28 },   // 0x36 '6'
  {  1081,  20,  29,  22,    1,  -28 },   // 0x37 '7'
  {  1154,  18,  29,  22,    2,  -28 },   // 0x38 '8'
  {  1220,  18,  29,  22,    2,  -28 },   // 0x39 '9'
  {  1286,   4,  22,  10,    3,  -21 },   // 0x3A ':'
  {  1297,   5,  28,   8,    1,  -21 },   // 0x3B ';'
  {  1315,  16,  18,  20,    1,  -21 },   // 0x3C '<'
  {  1351,  16,  11,  22,    3,  -18 },   // 0x3D '='
  {  1373,  16,  18,  21,    3,  -21 },   // 0x3E '>'
  {  1409,  16,  29,  19,    1,  -28 },   // 0x3F '?'
  {  1467,  32,  36,  36,    2,  -27 },   // 0x40 '@'
  {  1611,  26,  29,  26,    0,  -28 },   // 0x41 'A'
  {  1706,  20,  29,  25,    3,  -28 },   // 0x42 'B'
  {  1779,  22,  29,  26,    2,  -28 },   // 0x43 'C'
  {  1859,  21,  29,  26,    3,  -28 },   // 0x44 'D'
  {  1936,  18,  29,  23,    3,  -28 },   // 0x45 'E'
  {  2002,  18,  29,  22,    3,  -28 },   // 0x46 'F'
  {  2068,  22,  29,  27,    2,  -28 },   // 0x47 'G'
  {  2148,  22,  29,  29,    3,  -28 },   // 0x48 'H'
  {  2228,   3,  29,  11,    4,  -28 },   // 0x49 'I'
  {  2239,  18,  29,  22,    1,  -28 },   // 0x4A 'J'
  {  2305,  22,  29,  25,    3,  -28 },   // 0x4B 'K'
  {  2385,  18,  29,  22,    3,  -28 },   // 0x4C 'L'
  {  2451,  29,  29,  35,    3,  -28 },   // 0x4D 'M'
  {  2557,  22,  29,  29,    4,  -28 },   // 0x4E 'N'
  {  2637,  23,  29,  28,    2,  -28 },   // 0x4F 'O'
  {  2721,  21,  29,  25,    3,  -28 },   // 0x50 'P'
  {  2798,  23,  33,  28,    2,  -28 },   // 0x51 'Q'
  {  2893,  21,  29,  25,    3,  -28 },   // 0x52 'R'
  {  2970,  20,  29,  24,    2,  -28 },   // 0x53 'S'
  {  3043,  22,  29,  24,    1,  -28 },   // 0x54 'T'
  {  3123,  20,  29,  26,    3,  -28 },   // 0x55 'U'
  {  3196,  25,  29,  25,    0,  -28 },   // 0x56 'V'
  {  3287,  34,  29,  35,    1,  -28 },   // 0x57 'W'
  {  3411,  23,  29,  25,    1,  -28 },   // 0x58 'X'
  {  3495,  24,  29,  24,    0,  -28 },   // 0x59 'Y'
  {  3582,  20,  29,  24,    2,  -28 },   // 0x5A 'Z'
  {  3655,   7,  39,  11,    3,  -32 },   // 0x5B '['
  {  3690,  15,  31,  16,    1,  -28 },   // 0x5C '\'
  {  3749,   8,  39,  11,    0,  -32 },   // 0x5D ']'
  {  3788,  15,  14,  17,    1,  -28 },   // 0x5E '^'
  {  3815,  18,   3,  18,    0,    1 },   // 0x5F '_'
  {  3822,   8,   6,  12,    1,  -29 },   // 0x60 '`'
  {  3828,  18,  22,  22,    2,  -21 },   // 0x61 'a'
  {  3878,  17,  30,  22,    3,  -29 },   // 0x62 'b'
  {  3942,  18,  22,  21,    2,  -21 },   // 0x63 'c'
  {  3992,  18,  30,  23,    2,  -29 },   // 0x64 'd'
  {  4060,  18,  22,  21,    2,  -21 },   // 0x65 'e'
  {  4110,  13,  30,  14,    1,  -29 },   // 0x66 'f'
  {  4159,  18,  30,  22,    2,  -21 },   // 0x67 'g'
  {  4227,  16,  30,  22,    3,  -29 },   // 0x68 'h'
  {  4287,   4,  30,  10,    3,  -29 },   // 0x69 'i'
  {  4302,   8,  38,  10,   -1,  -29 },   // 0x6A 'j'
  {  4340,  17,  30,  20,    3,  -29 },   // 0x6B 'k'
  {  4404,   4,  30,  10,    3,  -29 },   // 0x6C 'l'
  {  4419,  29,  22,  35,    3,  -21 },   // 0x6D 'm'
  {  4499,  16,  22,  22,    3,  -21 },   // 0x6E 'n'
  {  4543,  19,  22,  23,    2,  -21 },   // 0x6F 'o'
  {  4596,  17,  30,  22,    3,  -21 },   // 0x70 'p'
  {  4660,  18,  30,  23,    2,  -21 },   // 0x71 'q'
  {  4728,  10,  22,  14,    3,  -21 },   // 0x72 'r'
  {  4756,  17,  22,  21,    2,  -21 },   // 0x73 's'
  {  4803,  12,  27,  13,    0,  -26 },   // 0x74 't'
  {  4844,  16,  22,  22,    3,  -21 },   // 0x75 'u'
  {  4888,  18,  22,  19,    0,  -21 },   // 0x76 'v'
  {  4938,  28,  22,  30,    1,  -21 },   // 0x77 'w'
  {  5015,  18,  22,  20,    1,  -21 },   // 0x78 'x'
  {  5065,  18,  30,  19,    0,  -21 },   // 0x79 'y'
  {  5133,  17,  22,  20,    2,  -21 },   // 0x7A 'z'
  {  5180,  12,  38,  14,    1,  -30 },   // 0x7B '{'
  {  5237,   3,  34,  10,    3,  -28 },   // 0x7C '|'
  {  5250,  12,  38,  14,    1,  -30 },   // 0x7D '}'
  {  5307,  23,   8,  27,    2,  -15 } }; // 0x7E '~'

const GFXfont roboto36pt7b = {
  (uint8_t  *)roboto36pt7bBitmaps,
  (GFXglyph *)roboto36pt7bGlyphs,
  0x20, 0x7E, 36 };//B

// Approx. 6002 bytes
