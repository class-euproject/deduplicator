#include "WebServer.h"
#include <iostream>

using namespace std;

namespace fog{
std::string out;

char* WebServer::doYourWork(char * msg, int msglen) {
    char *res = "Hello World!";

    cout << "WebServer::doYourWork()" << endl;
    out.append("HTTP/1.1 200 OK\n");
    out.append("Content-Type: text/plain\n");
    out.append("Content-Length: " + to_string (strlen(res)) + "\n");
    out.append("\n");
    out.append(res);
    
// Cache-Control: no-cache
// Pragma: no-cache
// Access-Control-Allow-Origin: *
// Date: Thu, 08 Oct 2020 12:10:38 GMT

    char * ret = (char *) out.c_str();
    cout << "buildResponse out is " << out << endl;
    cout << "buildResponse out.c_str() is " << out.c_str() << endl;
    cout << "buildResponse ret is " << ret << " and its length is " << strlen(ret)<< endl;
    
    return ret;
}
}
