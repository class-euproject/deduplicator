#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "Server.h"
#include <string>
#include <map>

using namespace std;

namespace fog {


class WebServer : public Server {

typedef int (WebServer::*pfunc)(string);
private:
    char responseBuffer[1024];
    char * buildResponse(int, char*);
    int parseQueryString(string);
    char * handleBus(string);
    char * handleOptions();

    map<string, pfunc> funcMap;

public:
    WebServer() : Server(80) { 
      funcMap["/api/bus"] = &WebServer::handleBus;
    }

    char * doYourWork(char *, int) override;
};
}

#endif /* WEB_SERVER_H */
