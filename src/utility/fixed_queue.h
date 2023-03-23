#pragma once

#include <Arduino.h>

template<typename T, uint16_t SIZE>
class FixedQueue {

    public:
        FixedQueue() {
            tail = 0;
            head = 0;
            count = 0;
        }

        // get the first item
        T pop();

        // get a pointer to the first item
        T* peek();

        // add an item if there is space in the queue
        void push(const T& item);

        // clear all items from queue
        inline void clear();

        // check if queue is full
        inline bool full();

        // check if queue is empty
        inline bool empty();

        // get the available space in the queue
        inline uint16_t available();

        // get the unavailable (used) space in the queue
        inline uint16_t unavailable();

        // get the max size of the queue
        inline uint16_t max_size();

    private:

        T queue[SIZE];

        uint16_t tail;
        uint16_t head;
        uint16_t count;
};

template<typename T, uint16_t SIZE>
T FixedQueue<T, SIZE>::pop() {
    if (empty()) return T();

    T result = queue[tail];

    count--;
    tail = (tail + 1) % SIZE;
    return result;
}

template<typename T, uint16_t SIZE>
T* FixedQueue<T, SIZE>::peek() {
    if (empty()) return nullptr;
    return &(queue[tail]);
}

template<typename T, uint16_t SIZE>
void FixedQueue<T, SIZE>::clear() {
    count = 0;
    tail = head = 0;
}

template<typename T, uint16_t SIZE>
void FixedQueue<T, SIZE>::push(const T& item) {
    if (full()) return;

    queue[head] = item;
    
    count++;
    head = (head + 1) % SIZE;
}

template<typename T, uint16_t SIZE>
bool FixedQueue<T, SIZE>::full() {
    return count == SIZE;
}

template<typename T, uint16_t SIZE>
bool FixedQueue<T, SIZE>::empty() {
    return count == 0;
}

template<typename T, uint16_t SIZE>
uint16_t FixedQueue<T, SIZE>::available() {
    return SIZE - count;
}

template<typename T, uint16_t SIZE>
uint16_t FixedQueue<T, SIZE>::unavailable() {
    return count;
}

template<typename T, uint16_t SIZE>
uint16_t FixedQueue<T, SIZE>::max_size() {
    return SIZE;
}

