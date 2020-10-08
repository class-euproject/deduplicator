#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "Server.h"

namespace fog {

class WebServer : public Server {

public:
    WebServer() : Server(80) { }

    void *doYourWork(void *) override;
};
}

#endif /* WEB_SERVER_H */
