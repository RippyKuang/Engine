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
#include <future>
#include <queue>

// modified from https://github.com/eglimi/cpptime

namespace Engine
{
    #define _s * 1e9
    #define _ms * 1e6
    #define _us * 1e3
    using handler_t = std::function<void()>;
    using clock = std::chrono::steady_clock;
    using timestamp = std::chrono::time_point<clock>;
    using duration = std::chrono::nanoseconds;

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
        std::vector<Time_event> time_events;
        std::stack<int> free_ids;
        bool done = false;

    public:
        int add(handler_t &&, const int dur = 0);
        bool remove(int id);
        void run();
        Timer(size_t threads=3) : m{}, cond{}, worker{}
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

    class ThreadPool {
        public:
            ThreadPool(size_t);
            template<class F, class... Args>
            auto enqueue(F&& f, Args&&... args)
                -> std::future<typename std::result_of<F(Args...)>::type>;
            ~ThreadPool();
        private:
           
            std::vector< std::thread > workers;
            std::queue< std::function<void()> > tasks;
             
            std::mutex queue_mutex;
            std::condition_variable condition;
            bool stop;
        };
          
        inline ThreadPool::ThreadPool(size_t threads)
            :   stop(false)
        {
            for(size_t i = 0;i<threads;++i)
                workers.emplace_back(
                    [this]
                    {
                        for(;;)
                        {
                            std::function<void()> task;
         
                            {
                                std::unique_lock<std::mutex> lock(this->queue_mutex);
                                this->condition.wait(lock,
                                    [this]{ return this->stop || !this->tasks.empty(); });
                                if(this->stop && this->tasks.empty())
                                    return;
                                task = std::move(this->tasks.front());
                                this->tasks.pop();
                            }
         
                            task();
                        }
                    }
                );
        }
         
        template<class F, class... Args>
        auto ThreadPool::enqueue(F&& f, Args&&... args)
            -> std::future<typename std::result_of<F(Args...)>::type>
        {
            using return_type = typename std::result_of<F(Args...)>::type;
         
            auto task = std::make_shared< std::packaged_task<return_type()> >(
                    std::bind(std::forward<F>(f), std::forward<Args>(args)...)
                );
                 
            std::future<return_type> res = task->get_future();
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
         
                if(stop)
                    throw std::runtime_error("enqueue on stopped ThreadPool");
         
                tasks.emplace([task](){ (*task)(); });
            }
            condition.notify_one();
            return res;
        }
         
        inline ThreadPool::~ThreadPool()
        {
            {
                std::unique_lock<std::mutex> lock(queue_mutex);
                stop = true;
            }
            condition.notify_all();
            for(std::thread &worker: workers)
                worker.join();
        }

}
