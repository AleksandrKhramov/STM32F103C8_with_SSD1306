//------------------------------------------------------------------------------
#ifndef _F10X16F_H
#define _F10X16F_H

#include "main.h"


// Шрифт моноширинный, 6х8 пикселей
#define f10x16_FLOAT_WIDTH         10
#define f10x16_FLOAT_HEIGHT        16

// Кол-во символов в таблице шрифта
#define f10x16f_NOFCHARS           256


// Функция возвращает указатель на подтаблицу символа Char
uint8_t *f10x16f_GetCharTable(uint8_t Char);

#endif 
