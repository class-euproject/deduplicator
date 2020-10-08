#include "WebServer.h"
#include <iostream>

using namespace std;

namespace fog{

char* WebServer::doYourWork(char * msg, int msglen) {
    cout << "WebServer::doYourWork()" << endl;
    
    cout << "Here is the message: " << msg << endl;
    
    return "I got your message";
}
}
