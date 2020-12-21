#include <Utilities.hpp>
#include <iostream>

Utilities::Utilities(){
    std::cout << "Constructor of Utilities" << std::endl;
}

Utilities::~Utilities(){
    /* Nix */
}

void Utilities::HelloWorld(){
    std::cout << " Hello World " << std::endl;
    actualHelloWorld();
}