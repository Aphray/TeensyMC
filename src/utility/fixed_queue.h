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

        // remove and return first item
        T pop();

        // remove and return first item with interrupts disabled
        T pop_no_interrupts();

        // get a pointer to the first item
        T* peek_first();

        // get a pointer to the last item
        T* peek_last();

        // add an item if there is space in the queue
        void push(const T& item);

        // add item to queue with interrupts disabled
        void push_no_interrupts(const T& item);

        // clear all items from queue
        inline void clear();

        // check if queue is full
        inline bool full();

        // check if queue is empty
        inline bool empty();

        // get the free space in the queue
        inline uint16_t free_space();

        // get the number of items in the queue
        inline uint16_t size();

        // get the max number of items that can be added to the queue
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
T FixedQueue<T, SIZE>::pop_no_interrupts() {
    noInterrupts();
    T result = pop();
    interrupts();
    return result;
}

template<typename T, uint16_t SIZE>
T* FixedQueue<T, SIZE>::peek_first() {
    if (empty()) return nullptr;
    return &(queue[tail]);
}

template<typename T, uint16_t SIZE>
T* FixedQueue<T, SIZE>::peek_last() {
    if (empty()) return nullptr;
    return &(queue[head]);
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
void FixedQueue<T, SIZE>::push_no_interrupts(const T& item) {
    noInterrupts();
    push(item);
    interrupts();
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
uint16_t FixedQueue<T, SIZE>::free_space() {
    return SIZE - count;
}

template<typename T, uint16_t SIZE>
uint16_t FixedQueue<T, SIZE>::size() {
    return count;
}

template<typename T, uint16_t SIZE>
uint16_t FixedQueue<T, SIZE>::max_size() {
    return SIZE;
}