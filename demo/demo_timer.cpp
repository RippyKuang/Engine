#include <iostream>
#include <unistd.h>
#include <timer.h>
#include <ctime>

using namespace Engine;

timestamp last_A_speak = clock::now();
timestamp last_B_speak = clock::now();

void foo()
{
    timestamp curr_A_speak = clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::microseconds>(curr_A_speak - last_A_speak).count() / 1000.0;
    last_A_speak = curr_A_speak;
    std::cout << "A speak at " << dur << std::endl;
}

void bar()
{
    timestamp curr_B_speak = clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::microseconds>(curr_B_speak - last_B_speak).count() / 1000.0;
    last_B_speak = curr_B_speak;
    std::cout << "B speak at" << dur << std::endl;
}

int main(int argc, char *argv[])
{
    Timer timer;
    int a = timer.add(foo, 100 * 1e3);
    int b = timer.add(bar, 50 * 1e3);
    sleep(10);
    timer.remove(b);
    while (1)
    {
    }

    return 0;
}