#ifndef IMESSAGELISTENER_H
#define IMESSAGELISTENER_H

#include "../masa_protocol/include/messages.hpp"

namespace fog {
class IMessageListener{

    public:
        virtual void OnMessageReceived(MasaMessage*) = 0;
};
}

#endif /* IMESSAGELISTENER_H */
