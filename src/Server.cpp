#include "Server.h"
#include <thread>
#include <iostream>

using namespace std;

namespace fog{

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

Server::Server(int port) {

    portno = port;
    
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    
    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
}

Server::~Server() {
    // Nothing to dispose
}

void Server::start() {
    if (pthread_create(&serverThrd, NULL, (THREADFUNCPTR) &Server::serverThrdFn, this)) 
        perror("could not create thread");
}

void Server::end() {
    pthread_join(serverThrd, NULL);

    close(newsockfd);
    close(sockfd);
}

void *Server::doYourWork(void * ptr) {
    cout << "Server::doYourWork()" << endl;
}

}
