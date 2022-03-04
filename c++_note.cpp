// g++ -std=c++11 *.cpp -o output.out


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





/* 
References
https://www.geeksforgeeks.org/references-in-c/
 */




/* 
https://www.tutorialspoint.com/cplusplus/cpp_member_operators.htm

The . (dot) operator and the -> (arrow) operator are used to reference individual members of classes, structures, and unions.

The dot operator is applied to the actual object. The arrow operator is used with a pointer to an object. For example, consider the following structure −

struct Employee {
   char first_name[16];
   int  age;
}  emp;


The (.) dot operator
To assign the value "zara" to the first_name member of object emp, you would write something as follows −
strcpy(emp.first_name, "zara");

The (->) arrow operator
If p_emp is a pointer to an object of type Employee, then to assign the value "zara" to the first_name member of object emp, you would write something as follows −
strcpy(p_emp->first_name, "zara");

The -> is called the arrow operator. It is formed by using the minus sign followed by a greater than sign.

Simply saying: To access members of a structure, use the dot operator. 
To access members of a structure through a pointer, use the arrow operator.
 */



/* 
http://www.cplusplus.com/forum/beginner/111502/

:: 
is the scope resolution operator, and allows you to statically traverse scopes such as namespaces and classes in order to reference the identifier you want.
 */



/* 
command click, go to definition

control - shift , go forward
control - , go back

 */



/* 
Three different ways to iterate vectors in C++ using for keyword
https://dev.to/vikramvee/three-different-ways-to-iterate-vectors-in-c-using-for-keyword-3871
Hello friends, Vectors in C++ are equivalent of List in C# and Java. Vectors are basically used to hold the elements of same type. More specifically it is a collection of elements. The elements could be of any primitive type like int, string, float or it could be any user defined type.

Please note that you should be using C++ 11 or above to use some of the features of the article.

Adding elements to Vector
In this article I will show you a small code snippet for different ways to iterate over the vectors in C++.
    vector<int> vec;
    for(int i = 0; i < 10 ; i++){
        vec.push_back(i);
    }
In the above code I have declared a variable of type vector named vec. And I am inserting 10 elements to the vector.

The elements will be inserted one after another in the increasing order of the index of the array. But we are not here to see how the elements are inserted, there are couple of more ways to do the same.

Iterate over a C++ vector
Since we have all the elements in the vector defined in the previous code snippet. What if we want to iterate over vector and find out about these items. There are three different ways we can iterate over the elements of vector in C++.

The first way to iterate over the elements is using the range for. It's new in C++ 11 and made the iteration even more attractive.
 for(auto item: vec){
        cout << item << endl;
    }
Basically in the above code, I am iterating over the vector using auto keyword. This will auto determine the type of the elements in the vector. It takes each and every element in the vector and proceeds for its task for that element. Here we are simply printing the element.

The second one is the traditional way we use in programming. By declaring a variable which will take care of the elements and size of the array. The variable will act as index for the vector elements.

In this approach we have to be bit careful as we should know about our vector. What is the size of the elements, how many iterations we want for the vector, the type of the elements in the vector, getting the elements using the index and so on.

Size returns unsigned integer that is the reason I have used the same for the variable deceleration. You can use integer as well but you will get a warning for the same.
  for(unsigned int i = 0; i < vec.size(); i++){
        cout << vec[i] << endl;
    }
The third way as shown in the below code snippet is use to iterators. There are methods provided by the C++ compiler to get the start and end iterator of the vector. The begin method takes the vector as the parameter and returns the beginning of the vector. The end of the vector is provided by end function. And there is a comparison which takes care of the end of the vector.

Instead of using the index notation or square brackets to get the element from the vector we use the reference of the iterator to get the current element.
 for(auto i = begin(vec); i  != end(vec); i++){
        cout << *i << endl;
    }
}
It all depends on the programmer which way he wants to iterate over the vector. But all the three ways mentioned above serve the same purpose. I would personally go for first approach due to ease of syntax and no need of adding an extra parameter.

Conclusion:
This was a short post to quickly review all the ways to iterate over a vector using the for keyword in C++. There are other ways as well to achieve the same using while and do..while. 
 
*/