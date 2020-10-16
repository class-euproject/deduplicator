#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "Server.h"
#include <string>
#include <map>
#include "IMessageListener.h"

using namespace std;

namespace fog {


class WebServer : public Server, public IMessageListener {

typedef char * (WebServer::*pfunc)(string);

private:
    char responseBuffer[1024];
    char * buildResponse(int, char* = NULL, char* = NULL);
    int parseQueryString(string);
    char * handleBus(string);
    char * handleOptions();
    map<int, MasaMessage*> _messages;

    map<string, pfunc> funcMap;

public:
    WebServer() : Server(80) { 
      funcMap["/api/bus"] = &WebServer::handleBus;
    }

    char * doYourWork(char *, int) override;
    void OnMessageReceived(MasaMessage *) override;
};
}

#endif /* WEB_SERVER_H */
