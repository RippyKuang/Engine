#include <coroutine>
#include <twist.h>
#include <type_traits>

#include <iostream>
#include <unistd.h>
#include <timer.h>

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

            std::suspend_always initial_suspend()
            {
                return {};
            };

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

    template <int _N>
    struct LinearGen
    {
        Matrix<double, 1, _N> start, end;

        duration dur;
        LinearGen(Matrix<double, 1, _N> s, Matrix<double, 1, _N> e, const size_t dur) : start(s), end(e), dur(dur) {}
        Matrix<double, 1, _N> operator()(duration elapsed)
        {
            return start + ((end - start) / dur.count()) * elapsed.count();
        }

        bool if_overtime(duration elapsed)
        {
            return elapsed > dur;
        }
    };

    template <typename F>
    Generator<std::invoke_result_t<F, duration>> sequence(F f)
    {

        timestamp start_time = clock::now();
        while (true)
        {
            auto elapsed = clock::now() - start_time;
            std::cout << elapsed.count() << std::endl;
            if (f.if_overtime(elapsed))
            {
                co_yield f.end;
                break;
            }
            co_yield std::invoke(f, elapsed);
        }
    }
}
