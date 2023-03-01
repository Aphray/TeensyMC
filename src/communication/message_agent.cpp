#include "message_agent.h"


_message_agent::_message_agent(Stream* stream_) {
    stream = stream_;
}

void _message_agent::post_queued_messages() {
    while (message_queue.size() > 0) {
            _message message = message_queue.front();
            stream->println(message.buffer);
            message_queue.pop();
        }
}