#pragma once

#include <Arduino.h>
#include <queue>

template<typename T, uint16_t SIZE>
class FixedQueue{
    private:
        queue<T> q;
    
    public:

        bool push(T element) {
            if (q.size() < SIZE) {
                q.push(element);
                return true;
            }
            return false;
        }
        
        T front() {
            return q.front();
        }
        
        void pop() {
            q.pop();
        }

        void clear() {
            q = {};
        }
        
        bool full() {
            return q.size() == SIZE;
        }
        
        bool empty() {
            return q.empty();
        }
        
        uint16_t size() {
            return q.size();
        }
                
        uint16_t max_size() {
            return SIZE;
        }

        uint16_t space_available() {
            return SIZE - q.size();
        }

};
