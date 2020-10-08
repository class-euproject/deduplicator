#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "Server.h"

namespace fog {

class WebServer : public Server {

private:
    char responseBuffer[1024];
    char *buildResponse(char});

public:
    WebServer() : Server(80) { }

    char * doYourWork(char *, int) override;
};
}

#endif /* WEB_SERVER_H */
