#include "message_agent.h"


MessageAgent::MessageAgent(Stream* stream_) {
    stream = stream_;
}

void MessageAgent::post_queued_messages() {
    while (message_queue.size() > 0) {
            Message message = message_queue.front();
            stream->println(message.buffer);
        }
}