#include <coroutine>
#include <twist.h>
namespace Engine
{
    template <typename T>
    struct Generator
    {
        class ExhaustedException : std::exception
        {
        };
        struct promise_type
        {
            T value;
            bool is_ready = false;

            std::suspend_always initial_suspend() { return {}; };

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

    template <int N>
    struct Linear : public Generator<double>
    {
        Matrix<double, 1, N> start;
        Matrix<double, 1, N> end;
        double dur;
        double cur;

    };

}
