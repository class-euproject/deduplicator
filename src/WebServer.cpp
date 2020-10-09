#include "WebServer.h"
#include <iostream>

using namespace std;

namespace fog{
std::string out;

char* WebServer::doYourWork(char * req, int reqlen) {

    

    char *res = "Hello World!";

    out.append("HTTP/1.1 200 OK\n");
    out.append("Cache-Control: no-cache\n");
    out.append("Pragma: no-cache\n");
    out.append("Access-Control-Allow-Origin: *\n");
    out.append("Content-Type: text/plain\n");
    out.append("Content-Length: " + to_string (strlen(res)) + "\n");
    out.append("\n");
    out.append(res);
    return (char *) out.c_str();
}
}
