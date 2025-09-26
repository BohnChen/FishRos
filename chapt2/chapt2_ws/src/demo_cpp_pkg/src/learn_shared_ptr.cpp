#include <iostream>
#include <memory>

int main()
{
    auto p1 = std::make_shared<std::string>("This is a str.");
    std::cout << "p1 count is : " << p1.use_count() << ", point to :" << p1.get()
        << std::endl;
    auto p2 = p1;
    std::cout << "p1 count is : " << p1.use_count() << ", point to : " << p1.get() 
        << std::endl;
    std::cout << "p2 count is : " << p2.use_count() << ", point to : " << p2.get() 
        << std::endl;

    p1.reset();
    std::cout << "p1 count is : " << p1.use_count() << ", point to : " << p1.get() 
        << std::endl;
    std::cout << "p2 count is : " << p2.use_count() << ", point to : " << p2.get() 
        << std::endl;
    std::cout << "the resoure p2 point to is " << p2->c_str() << std::endl;
}
