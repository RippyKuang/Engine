#include <iostream>
#include <unistd.h>
#include <trajectory.h>

template<typename T>
struct Generator
{
    class ExhaustedException : std::exception
    {
    };
    struct promise_type
    {
        T value;
        bool is_ready = false;

        std::suspend_never initial_suspend() { return {}; };

        std::suspend_always final_suspend() noexcept { return {}; }

        void unhandled_exception() {}

        Generator get_return_object()
        {
            return Generator{std::coroutine_handle<promise_type>::from_promise(*this)};
        }

        std::suspend_always yield_value(T value)
        {
            this->value = value;
            this->is_ready = true;
            return {};
        }

        void return_void() {}
    };
    std::coroutine_handle<promise_type> handle;

    bool has_next()
    {
        if (!handle || handle.done())
            return false;

        if (!handle.promise().is_ready)
            handle.resume();

        if (handle.done())
            return false;
        else
            return true;
    }

    T next()
    {
        if (has_next())
        {
            handle.promise().is_ready = false;
            return handle.promise().value;
        }
        else
        {
            throw ExhaustedException();
        }
    }
    ~Generator()
    {
        handle.destroy();
    }
};

Generator<int> sequence()
{
    int i = 0;
    while (true)
    {
        co_yield  i++;
    }
}

int main(int argc, char *argv[])
{
    auto gen = sequence();
    for (int i = 0; i < 5; ++i)
    {
        std::cout << gen.next() << std::endl;
    }
    return 0;
}