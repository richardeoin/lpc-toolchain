/* 
 * Defines safe debug functions that won't freeze the processor
 * Copyright (C) 2013  Richard Meadows
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdarg.h>
#include <LPC11Uxx.h>

/* CoreDebug registers are not accesible from processor in Cortex-M0 */
// CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk
#define DEBUG_ACTIVE()    (1)

void _debug_putchar(char c) {
  if (DEBUG_ACTIVE()) {
    putchar(c);
  }
}
void _debug_puts(const char* s) {
  if (DEBUG_ACTIVE()) {
    puts(s);
  }
}
void _debug_printf(const char *format, ...) {
  if (DEBUG_ACTIVE()) {
    va_list args;

    va_start(args, format);
    vprintf(format, args);
  }
}
