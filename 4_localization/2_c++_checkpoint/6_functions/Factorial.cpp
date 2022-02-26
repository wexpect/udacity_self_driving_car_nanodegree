#include "Factorial.h"

#include <iostream>

int Factorial(int n){
    if (n >= 2){
        return n * Factorial(n-1);
    }
    else {
        return 1;
    }
}