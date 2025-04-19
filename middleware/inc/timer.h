#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <set>
#include <stack>
#include <vector>
#include <thread>
#include <algorithm>

// modified from https://github.com/eglimi/cpptime

namespace Engine
{
    using handler_t = std::function<void()>;
    using clock = std::chrono::steady_clock;
    using timestamp = std::chrono::time_point<clock>;
    using duration = std::chrono::microseconds;

    struct Event
    {
        int id;
        duration period;
        timestamp start;
        handler_t handler;
        bool valid;
        Event()
            : id(0), period(duration::zero()), handler(nullptr), valid(false)
        {
        }
        template <typename Func>
        Event(int id, timestamp start, duration period, Func &&handler)
            : id(id), start(start), period(period), handler(std::forward<Func>(handler)), valid(true)
        {
        }
        Event(Event &&r) = default;
        Event &operator=(Event &&ev) = default;
        Event(const Event &r) = delete;
        Event &operator=(const Event &r) = delete;
    };

    struct Time_event
    {
        timestamp next;
        int ref;
    };

    class Timer
    {
    private:
        std::mutex m;
        std::condition_variable cond;
        std::thread worker;
        std::vector<Event> events;
        std::multiset<Time_event> time_events;
        std::stack<int> free_ids;
        bool done = false;

    public:
        int add(handler_t &&, const int dur = 0);
        bool remove(int id);
        void run();
        Timer() : m{}, cond{}, worker{}
        {
            std::unique_lock<std::mutex> lock(m);
            done = false;
            worker = std::thread([this]
                                 { run(); });
        }
        ~Timer()
        {
            std::unique_lock<std::mutex> lock(m);
            done = true;
            lock.unlock();
            cond.notify_all();
            worker.join();
        }
    };

}
