#include "WebServer.h"
#include <iostream>

using namespace std;

namespace fog{
std::string out;
bool done =false;

char* WebServer::doYourWork(char * req, int reqlen) {
    out = "";
    char *res = "Hello World!";
    if(!done) {
    out.append("HTTP/1.1 200 OK\n");
    out.append("Content-Length: 0\n");
    out.append("Connection: keep-alive\n");
    out.append("Access-Control-Allow-Origin: *\n");
    out.append("Access-Control-Allow-Headers: *\n");
    out.append("Access-Control-Allow-Methods: POST, GET, OPTIONS, DELETE\n");
    out.append("\n");
    }
    else {
    out.append("HTTP/1.1 200 OK\n");
    out.append("Cache-Control: no-cache\n");
    out.append("Pragma: no-cache\n");
    out.append("Access-Control-Allow-Origin: *\n");
    out.append("Access-Control-Allow-Headers: *\n");
    out.append("Content-Type: text/plain\n");
    out.append("Content-Length: " + to_string (strlen(res)) + "\n");
    out.append("\n");
    out.append("Ciao\n");
    out.append("\n");
    out.append(res);
    }
    done = ! done;
    return (char *) out.c_str();
}
}
