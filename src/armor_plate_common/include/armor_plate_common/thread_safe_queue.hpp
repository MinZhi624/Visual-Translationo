#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>
#include <functional>
#include <iostream>
#include <chrono>

/*
    来自sp2025的thread_safe_queue代码
*/
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue {
private :
    std::queue<T> queue_;
    size_t max_size_;
    std::function<void ()> full_handler_; //队列满时的回调函数
    std::mutex mtx_;

    /*
        这里是生产者-消费者的同步机制
        当消费者线程在队列为空时等待。
        当生产者来数据才唤醒消费者线程。
    */
    std::condition_variable not_empty_condition_;
public:
    ThreadSafeQueue(
        size_t max_size, std::function<void(void)> full_handler = [] {}) :
        max_size_(max_size),
        full_handler_(full_handler)
    {}

    void push(const T & value)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        if (queue_.size() >= max_size_) {
            if (PopWhenFull) {
                queue_.pop();
            } else {
                full_handler_();
                return;
            }
        }
        queue_.push(value);
        not_empty_condition_.notify_all();
    }

    void push(T && value)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        if (queue_.size() >= max_size_) {
            if (PopWhenFull) {
                queue_.pop();
            } else {
                full_handler_();
                return;
            }
        }
        queue_.push(std::move(value));
        not_empty_condition_.notify_all();
    }
    void pop(T & value)
    {
        std::unique_lock<std::mutex> lock(mtx_);

        /*
            [this]{return !queue_.empty();}
            用来防止虚假唤醒.
            原因：
            操作系统可能会无缘无故的唤醒线程。
        */
        not_empty_condition_.wait(lock, [this]{return !queue_.empty();});
        if (queue_.empty()) {
            std::cerr << "无法从空的队列中弹出元素" << std::endl;
            return;
        }
        value = queue_.front();
        queue_.pop();
    }

    T pop()
    {
        std::unique_lock<std::mutex> lock(mtx_);
        not_empty_condition_.wait(lock, [this]{return !queue_.empty();});
        T value = std::move(queue_.front());
        queue_.pop();
        return std::move(value);
    }

    T front()
    {
        std::unique_lock<std::mutex> lock(mtx_);
        not_empty_condition_.wait(lock, [this]{return !queue_.empty();});
        return queue_.front();
    }

    T back()
    {
        std::unique_lock<std::mutex> lock(mtx_);
        if (queue_.empty()) {
            std::cerr << "无法从空的队列中拿到最后的元素" << std::endl;
            return T{};
        }
        return queue_.back();
    }

    bool pop(T & value, std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        if (!not_empty_condition_.wait_for(lock, timeout, [this]{return !queue_.empty();})) {
            return false;
        }
        value = std::move(queue_.front());
        queue_.pop();
        return true;
    }
    bool empty()
    {
        std::unique_lock<std::mutex> lock(mtx_);
        return queue_.empty();
    }
    void clear()
    {
        std::unique_lock<std::mutex> lock(mtx_);
        while(!queue_.empty()) {
            queue_.pop();
        }
        not_empty_condition_.notify_all();
    }
};
