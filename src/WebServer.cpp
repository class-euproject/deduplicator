#include "WebServer.h"
#include <iostream>

using namespace std;

namespace fog{
std::string out;

char *WebServer::buildResponse(char *msg) {
    out.append("HTTP/1.1 200 OK\n");
// Cache-Control: no-cache
// Pragma: no-cache
// Content-Length: 10
// Content-Type: application/json; charset=utf-8
// Expires: -1
// Server: Microsoft-IIS/10.0
// Access-Control-Allow-Origin: *
// X-AspNet-Version: 4.0.30319
// Request-Context: appId=cid-v1:85028693-b879-42f6-808c-0e8e562f5246
// Access-Control-Expose-Headers: Request-Context
// X-Powered-By: ASP.NET
// Set-Cookie: ARRAffinity=166775b745e660bdb8a82a11166e31b9829590a3c9a8a411718e1a2174d19d20;Path=/;HttpOnly;Secure;Domain=cloudapi-dev1.iotty.com
// Set-Cookie: ARRAffinitySameSite=166775b745e660bdb8a82a11166e31b9829590a3c9a8a411718e1a2174d19d20;Path=/;HttpOnly;SameSite=None;Secure;Domain=cloudapi-dev1.iotty.com
// Date: Thu, 08 Oct 2020 12:10:38 GMT
   char * ret = (char *) out.c_str();
   cout << "buildResponse out is " << out << endl;
   cout << "buildResponse out.c_str() is " << out.c_str() << endl;
   cout << "buildResponse ret is " << ret << " and its length is " << strlen(ret)<< endl;
}

char* WebServer::doYourWork(char * msg, int msglen) {
    cout << "WebServer::doYourWork()" << endl;
    
    cout << "Here is the message: " << msg << endl;
    char * res = buildResponse("I got your message");
    cout << "res is " << res << endl;
}
}
