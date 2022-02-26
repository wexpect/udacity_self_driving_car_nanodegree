#include "Doubler.h"

#include <iostream>

int main(){
    int n = 1;

    std::cout << "Before " << n << std::endl;

    Doubler(n);

    std::cout << "After " << n << std::endl;
}