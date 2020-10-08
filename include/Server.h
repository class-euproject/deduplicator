#ifndef SERVER_H
#define SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>

namespace fog {

typedef void * (*THREADFUNCPTR)(void *);

class Server {

private:
    int sockfd, newsockfd;
    struct sockaddr_in serv_addr;
    int portno;
    pthread_t serverThd;
    
    const static long BUFSIZE = 256;
    char buffer[BUFSIZE];
    
public:
    Server(int port);
    ~Server();
    void start();
    void end();
    
    virtual void doYourWork() {

    }
};
}

#endif /* SERVER_H */
