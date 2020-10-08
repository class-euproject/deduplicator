#pragma once

namespace fog {
class WebServer {
public:
    WebServer();
    ~WebServer();
    void start();
    void end();
};

}