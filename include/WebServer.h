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
    char _responseBuffer[1024];
    char * buildResponse(int, char* = NULL, char* = NULL);
    int parseQueryString(string);
    char * handleBus(string);
    char * handleOptions();
    map<int, MasaMessage*> _messages;
    map<int, int> _busSizes;

    map<string, pfunc> _funcMap;

public:
    WebServer() : Server(80) { 
      _funcMap["/api/bus"] = &WebServer::handleBus;
      _busSizes[171] = 91;
    }

    char * doYourWork(char *, int) override;
    void OnMessageReceived(MasaMessage *) override;
};
}

#endif /* WEB_SERVER_H */
