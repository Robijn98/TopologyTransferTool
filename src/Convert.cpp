#include "../include/Convert.h"
#include <fstream>

Convert::file(str file)
{
}



void Convert::open_file(std::string file)
{
    std::string line;
    std::ifstream input(file);

    if (input.is_open())
    {
        while (getline(input, line))
        {
            output << line << std::endl;
        }
        input.close();
        output.close();
    }
    else
    {
        std::cerr << "Unable to open file" << std::endl;
    }
}