#include <timer.h>
namespace Engine
{

    inline bool operator<(const Time_event &l, const Time_event &r)
    {
        return l.next < r.next;
    }

    void Timer::run()
    {

        std::unique_lock<std::mutex> lock(m);

        while (!done)
        {
            if (events.empty())
                cond.wait(lock);
            else
            {
                Time_event te = *time_events.begin();
                if (clock::now() >= te.next)
                {

                    time_events.erase(time_events.begin());
                    lock.unlock();
                    events[te.ref].handler();
                    lock.lock();

                    if (events[te.ref].valid && events[te.ref].period.count() > 0)
                    {
                        te.next += events[te.ref].period;
                        time_events.emplace_back(te);
                    }
                    else
                    {
                        events[te.ref].valid = false;
                        events[te.ref].handler = nullptr;
                        free_ids.push(te.ref);
                    }
                }
            }
        }
    }

    bool Timer::remove(int id)
    {
        std::unique_lock<std::mutex> lock(m);
        if (events.size() == 0 || events.size() <= id)
        {
            return false;
        }
        events[id].valid = false;
        events[id].handler = nullptr;
        auto it = std::find_if(time_events.begin(), time_events.end(),
                               [&](const Time_event &te)
                               { return te.ref == id; });
        if (it != time_events.end())
        {
            free_ids.push(it->ref);
            time_events.erase(it);
        }
        lock.unlock();
        cond.notify_one();
        return true;
    }

    int Timer::add(handler_t &&handler, const int dur)
    {
        std::unique_lock<std::mutex> lock(m);
        int id = 0;
        timestamp when = clock::now();
        if (free_ids.empty())
        {
            id = events.size();
            Event e(id, when, duration(dur), std::move(handler));
            events.push_back(std::move(e));
        }
        else
        {
            id = free_ids.top();
            free_ids.pop();
            Event e(id, when, duration(dur), std::move(handler));
            events[id] = std::move(e);
        }
        time_events.emplace_back(Time_event{when, id});
        lock.unlock();
        cond.notify_one();
        return id;
    }
}