#ifndef THREADSAFEQUEUE_H
#define THREADSAFEQUEUE_H

#include <QMutex>
#include <QMutexLocker>
#include <QQueue>
#include <QtGlobal>

// 最小线程安全队列。
// 当前阶段优先保证骨架清晰和便于维护，后续如果需要追求更高吞吐，
// 再替换成环形缓冲区即可。
template <typename T>
class ThreadSafeQueue
{
public:
    void push(const T &value)
    {
        QMutexLocker locker(&mutex_);
        queue_.enqueue(value);
    }

    bool tryPop(T &value)
    {
        QMutexLocker locker(&mutex_);
        if (queue_.isEmpty()) {
            return false;
        }
        value = queue_.dequeue();
        return true;
    }

    void clear()
    {
        QMutexLocker locker(&mutex_);
        queue_.clear();
    }

    qsizetype size() const
    {
        QMutexLocker locker(&mutex_);
        return queue_.size();
    }

    bool empty() const
    {
        QMutexLocker locker(&mutex_);
        return queue_.isEmpty();
    }

private:
    mutable QMutex mutex_;
    QQueue<T> queue_;
};

#endif // THREADSAFEQUEUE_H
