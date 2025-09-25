#include <fmt/core.h>
#include <string>
using namespace std;

int main() 
{
    fmt::print("Hello, {}!\n", "World");

    string message = fmt::format("The answer is {}.", 42);
    fmt::print("{}\n", message);

    return 0;
}
