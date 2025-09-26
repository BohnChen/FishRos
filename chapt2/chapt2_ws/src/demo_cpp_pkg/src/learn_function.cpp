#include <iostream>
#include <functional>


void save_with_free_fun(const std::string &file_name)
{
    std::cout << "called the free function, save: " << file_name << std::endl;
}

class FileSave
{
public:
    void save_with_member_fun(const std::string &file_name)
    {
        std::cout << "called the member function, Save: " << file_name << std::endl;
    };
};


int main () 
{
    FileSave file_save;
    auto save_with_lambda_fun = [](const std::string &file_name) -> void
    {
        std::cout << "called the Lambda function , Save: " << file_name << std::endl;
    };
    // put the free function to the function object
    std::function<void(const std::string &)> save1 = save_with_free_fun;
    // put the Lambda to the function object
    std::function<void(const std::string &)> save2 = save_with_lambda_fun;
    // put the member function to function object
    std::function<void(const std::string &)> save3 = std::bind(&FileSave::save_with_member_fun,
                                                               &file_save, std::placeholders::_1);
    // Regardless of the function,a unified calling method can be used
    save1("file.txt");
    save2("file.txt");
    save3("file.txt");

    return 0;
}
