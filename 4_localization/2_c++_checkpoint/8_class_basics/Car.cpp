#include "Car.h"

#include <stdlib.h>
#include <time.h>
#include <iostream>


Car::Car() {
    srand(time(NULL));

    in_working_condition_ = true;
}

void Car::wearAndTear() {
    int condition = rand() % 10;
    condition >= 5? in_working_condition_ = true : in_working_condition_ = false;
}

bool Car::drive(){
    bool didDrive = false;

    if(in_working_condition_){
        std::cout << "Driving!" << std::endl;
        wearAndTear();
        didDrive = true;
    }
    else {
        std::cout << "Broken down. Please fix" << std::endl;
        didDrive = false;
    }

    return didDrive;
}

void Car::fix(){
    in_working_condition_ = true;
    std::cout << "Fixed!" << std::endl;
}
