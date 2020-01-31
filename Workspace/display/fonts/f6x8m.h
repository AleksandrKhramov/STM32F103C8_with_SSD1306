//------------------------------------------------------------------------------
#ifndef _F6X8M_H
#define _F6X8M_H

#include <main.h>


// Шрифт моноширинный, 6х8 пикселей
#define f6x8_MONO_WIDTH         6
#define f6x8_MONO_HEIGHT        8

// Кол-во символов в таблице шрифта
#define f6x8_NOFCHARS           256


// Функция возвращает указатель на подтаблицу символа Char
uint8_t *f6x8m_GetCharTable(uint8_t Char);

#endif 
