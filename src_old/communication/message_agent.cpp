#include "message_agent.h"


_message_agent::_message_agent(Stream* stream_) {
    stream = stream_;
}

void _message_agent::post_queued_messages() {
    while (message_queue.size()) {
            _message message = message_queue.pop();
            stream->println(message.buffer);
        }
}