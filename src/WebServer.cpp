#include "WebServer.h"

namespace fog{

void error(const char *msg)
{
    perror(msg);
    exit(1);
}



WebServer::WebServer(int port) {

    portno = port;
    
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    
    bzero((char *) &serv_addr, sizeof(serv_addr));

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
}

WebServer::~WebServer() {
    // Nothing to dispose
}

void WebServer::start() {
    struct sockaddr_in cli_addr;
    socklen_t clilen = sizeof(cli_addr);
    if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
        error("ERROR on binding");
    listen(sockfd,5);
    
    newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);

    if (newsockfd < 0) 
        error("ERROR on accept");
    bzero(buffer,BUFSIZE);
    int n = read(newsockfd,buffer,BUFSIZE - 1);
    if (n < 0)
        error("ERROR reading from socket");
    printf("Here is the message: %s\n",buffer);
    n = write(newsockfd,"I got your message",18);
    if (n < 0)
        error("ERROR writing to socket");
}

void WebServer::end() {
    close(newsockfd);
    close(sockfd);
}

}
