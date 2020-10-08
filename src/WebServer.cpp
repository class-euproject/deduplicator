#include "WebServer.h"
#include <iostream>

using namespace std;

namespace fog{

void *WebServer::doYourWork(void * ptr) {
    cout << "WebServer::doYourWork()" << endl;
    // struct sockaddr_in cli_addr;
    // socklen_t clilen = sizeof(cli_addr);
    // if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
    //     error("ERROR on binding");
    // listen(sockfd,5);
    
    // newsockfd = accept(sockfd, (struct sockaddr *) &cli_addr, &clilen);

    // if (newsockfd < 0) 
    //     error("ERROR on accept");
    // bzero(buffer,BUFSIZE);
    // int n = read(newsockfd,buffer,BUFSIZE - 1);
    // if (n < 0)
    //     error("ERROR reading from socket");
    // printf("Here is the message: %s\n",buffer);
    // n = write(newsockfd,"I got your message",18);
    // if (n < 0)
    //     error("ERROR writing to socket");
}
}
