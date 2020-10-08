#ifndef WEB_SERVER_H
#define WEB_SERVER_H

namespace fog {
class WebServer {
public:
    WebServer(int port);
    ~WebServer();
    void start();
    void end();
};

}

#endif
