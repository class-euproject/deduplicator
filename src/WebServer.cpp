#include "WebServer.h"
#include <iostream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <map>

using namespace std;

namespace fog{
    
typedef int (*pfunc)(string);

map<string, pfunc> funcMap;
map<string, string> query;
string retval;


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

int WebServer::handleBus(string s) {
    cout << "handleBus(\"" << s << "\")" << endl;

    retval = "";
    char *res = "Hello World!";
    retval.append("HTTP/1.1 200 OK\n");
    retval.append("Cache-Control: no-cache\n");
    retval.append("Pragma: no-cache\n");
    retval.append("Access-Control-Allow-Origin: *\n");
    retval.append("Access-Control-Allow-Headers: *\n");
    retval.append("Content-Type: text/plain\n");
    retval.append("Content-Length: " + to_string (strlen(res)) + "\n");
    retval.append("\n");
    retval.append(res);
    
    return 200;
        
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

int WebServer::handleOptions() {
    retval = "";
    retval.append("HTTP/1.1 200 OK\n");
    retval.append("Content-Length: 0\n");
    retval.append("Connection: keep-alive\n");
    retval.append("Access-Control-Allow-Origin: *\n");
    retval.append("Access-Control-Allow-Headers: *\n");
    retval.append("Access-Control-Allow-Methods: GET, OPTIONS\n");
    retval.append("\n");
}

char* WebServer::doYourWork(char * req, int reqlen) {
    int r = process(req);
    
    if(ret != 200) {
        retval.append("HTTP/1.1 500 Internal error\n"); // TODO
        retval.append("Cache-Control: no-cache\n");
        retval.append("Pragma: no-cache\n");
        retval.append("Access-Control-Allow-Origin: *\n");
        retval.append("Access-Control-Allow-Headers: *\n");
        retval.append("Content-Length: 0\n");
        retval.append("\n");
    }

    return retval.c_str();
}

int WebServer::process(char * req) {
    retval = "";
    vector<string> strs;
    
    cout << "Received request : " << req << endl << endl;
    
    boost::split(strs, req, boost::is_any_of("\n"));

    if(strs.size() < 1) {
        retval = "Malformed request header";
        return 400; // Malformed header
    }
    
    boost::split(strs, strs.at(0), boost::is_any_of(" "));

    if(strs.size() != 3) {
        retval = "Malformed request header";
        return 400; // Malformed header
    }

    if(strs.at(0) == "OPTIONS") {
        return handleOptions();
    }

    else if(strs.at(0) == "GET")  {
        // Try to identify the EP
        
        boost::split(strs, strs.at(1), boost::is_any_of("?"));

        if(strs.size() < 1 && strs.at(0) != "") {
            retval = "Malformed request header";
            return 400; // Malformed header
        }
    
        string querystring = "";
        // 0 -> Endpoint; 1-> query string
        if(strs.size() > 1)
            querystring = strs.at(1);
            
        pfunc f = funcMap[strs.at(0)];
        if(f == NULL) {
            retval = "Endpoint '" + strs.at(0) + "' not found";
            return 404; // Ep not found
        }

        return (*f)(querystring);
    }
    else {
        retval = "Unsupported method '" + strs.at(0) + "'";
        return 405; // Unsupported
    }
}
}
