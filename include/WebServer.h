#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

namespace fog {
class WebServer {

private:
    int sockfd, newsockfd;
    struct sockaddr_in serv_addr;
    int portno;
    
    const static long BUFSIZE = 256;
    char buffer[BUFSIZE];
    
public:
    WebServer(int port);
    ~WebServer();
    void start();
    void end();
};

}

#endif /* WEB_SERVER_H */
