#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "Server.h"

namespace fog {

class WebServer : Server {

public:
    WebServer() : Server(80) { }

    void doYourWork() override;
};
}

#endif /* WEB_SERVER_H */
