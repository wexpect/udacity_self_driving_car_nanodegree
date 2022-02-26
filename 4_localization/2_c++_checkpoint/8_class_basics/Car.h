#ifndef CAR_H
#define CAR_H

/* 
By default, all members of a class declared with the class keyword have private access for all its members. 
Therefore, any member that is declared before any other access specifier has private access automatically. 
For example:
class Rectangle {
    int width, height;
  public:
    void set_values (int,int);
    int area (void);
} rect;
 */

class Car {
    private:
        //  the trailing _ on in_working_condition_ is common tactic for designating private properties in C++
        bool in_working_condition_;
    public:
        Car();
        void wearAndTear();
        bool drive();
        void fix();    
};

#endif