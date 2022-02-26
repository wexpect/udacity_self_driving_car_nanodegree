/* 
#ifndef and #define are known as header guards. 
Their primary purpose is to prevent C++ header files 
from being included multiple times.


https://docs.microsoft.com/en-us/cpp/preprocessor/hash-ifdef-and-hash-ifndef-directives-c-cpp?view=msvc-170
#ifdef identifier
#ifndef identifier

These directives are equivalent to:
#if defined identifier
#if !defined identifier

The #ifndef directive checks for the opposite of the condition checked by #ifdef. 
If the identifier hasn't been defined, or if its definition has been removed with #undef, 
the condition is true (nonzero). Otherwise, the condition is false (0).
 */

#ifndef PRINTSTRING_H
#define PRINTSTRING_H

#include <string>

void PrintString(std::string, int);

#endif