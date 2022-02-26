/* 
C++ classes (and often function prototypes) are normally split up into two files. 
The header file has the extension of .h and contains class definitions and functions. 
The implementation of the class goes into the .cpp file. 
 */

/* 
Order of include:
from local to global, each subsection in alphabetical order, i.e.:

- h file corresponding to this cpp file (if applicable)
- headers from the same component,
- headers from other components, or third party
- system headers.
 */

#include "PrintString.h"

#include <iostream>
#include <string>

void PrintString(std::string str, int n){
    for (int i = 0; i < n; i++){
        std::cout << str << std::endl;
    }
}