#include "WebServer.h"
#include <iostream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/assign.hpp>
#include <vector>
#include <map>

using namespace std;
using namespace boost::assign;

namespace fog{
    
string retval; // Buffer for HTTP response
map<string, string> query; // We store the query
map<int, string> statuses = map_list_of (200, "OK") (400, "Bad request") (404, "Not found") (405, "Unsupported") (500, "Internal error");

int WebServer::parseQueryString(string querystring) {
    vector<string> split, split2;
    boost::split(split, querystring, boost::is_any_of("&"));
    query.clear();
    for(int i=0; i< split.size(); i++) {
        if(split.at(i) != "") {
            cout << "[" << i << "] " << split.at(i) << endl;
            boost::split(split2, split.at(i), boost::is_any_of("="));
            string val = "";
            if(split2.size() != 2) {
                retval = "Wrong parameter '" + split.at(i) + "'";
                return 400;
            }
            query[split2.at(0)] = split2.at(1);
        }
    }
    return 0;
}

char * WebServer::buildResponse(int status, char * res) {
    retval = "HTTP/1.1 " + to_string(status) + " " + statuses[status] +"\n";
    retval.append("Cache-Control: no-cache\n");
    retval.append("Pragma: no-cache\n");
    retval.append("Access-Control-Allow-Origin: *\n");
    retval.append("Access-Control-Allow-Headers: *\n");
    retval.append("Content-Type: text/plaini\n");
    if(res == NULL)
        retval.append("Content-Length: 0\n");
    else
        retval.append("Content-Length: " + to_string (strlen(res)) + "\n");
    retval.append("\n");

    if(res != NULL)
        retval.append(res);

    return (char *) retval.c_str();
}

char* WebServer::handleBus(string s) {
    cout << "handleBus(\"" << s << "\")" << endl;

    return buildResponse(200, "Hello World!");
        
    // if(int err = parseQueryString(s))
    //     return err;

    // if(query.size() == 0 || query["id"] == "") {
    //     retval = "'id' param not specified";
    //     return 400;
    // }

    // retval = "";
    
    // retval.append("[\n");
    // retval.append("\t{ \"cam_idx\" : " + to_string (107) + " },\n");
    // retval.append("\t{ \"t_stamp_ms\" : " + to_string (16353) + " },\n");
    // retval.append("\t{ \"num_objects\" : " + to_string (10) + " },\n");
    // retval.append("]");
    // return 200;
    // for(map<string, string>::iterator it = query.begin(); it != query.end(); ++it)
    //     std::cout << it->first << " :: " << it->second << std::endl;

    // retval = "Internal error";
    // return 500;
}

char* WebServer::handleOptions() {
    retval = "HTTP/1.1 200 OK\n";
    retval.append("Content-Length: 0\n");
    retval.append("Connection: keep-alive\n");
    retval.append("Access-Control-Allow-Origin: *\n");
    retval.append("Access-Control-Allow-Headers: *\n");
    retval.append("Access-Control-Allow-Methods: GET, OPTIONS\n");
    retval.append("\n");
    return (char *) retval.c_str();
}

char* WebServer::doYourWork(char * req, int reqlen) {
    retval = "";
    vector<string> strs;
    
    boost::split(strs, req, boost::is_any_of("\n"));

    if(strs.size() < 1)
        return buildResponse(400);
    
    boost::split(strs, strs.at(0), boost::is_any_of(" "));

    if(strs.size() != 3)
        return buildResponse(400);

    if(strs.at(0) == "OPTIONS")
        return handleOptions();

    else if(strs.at(0) == "GET")  {        
        boost::split(strs, strs.at(1), boost::is_any_of("?"));

        if(strs.size() < 1 && strs.at(0) != "")
        return buildResponse(400);
    
        string querystring = "";
        // 0 -> Endpoint; 1-> query string
        if(strs.size() > 1)
            querystring = strs.at(1);
            
        pfunc f = funcMap[strs.at(0)];
        if(f == NULL) {
            cout << "Handler for endpoint '" << strs.at(0) << "' not found" << endl;
            return buildResponse(404);
        }

        return (this->*f)(querystring);

    }
    else
        return buildResponse(405);
}
}
