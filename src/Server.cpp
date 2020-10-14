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

    close(sockfd);
}

void *Server::serverThrdFn(void * ptr) {

    struct sockaddr_in cli_addr;
    socklen_t clilen = sizeof(cli_addr);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
        error("ERROR on binding");

    while(1) {
        listen(sockfd,5);
        
        newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);

        if (newsockfd < 0) 
            error("ERROR on accept");
        bzero(buffer,BUFSIZE);
        int n = read(newsockfd,buffer,BUFSIZE - 1);
        if (n < 0)
            error("ERROR reading from socket");

	    //cout << "REQUEST:" << endl << buffer << endl << endl;

        char * ret = doYourWork(&buffer[0], n);
        cout << "RESPONSE:" << endl << ret << endl << endl;

        n = write(newsockfd, ret, strlen(ret));
        if (n < 0)
            error("ERROR writing to socket");

        close(newsockfd);
    }
}

char* Server::doYourWork(char * req, int reqlen) {
    cout << "Server::doYourWork()" << endl;
}
}
