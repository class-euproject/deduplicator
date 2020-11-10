#include "WebServer.h"
#include <iostream>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/assign.hpp>
#include <vector>
#include <map>
#include "../masa_protocol/include/messages.hpp"
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace boost::assign;

namespace fog{
    
string retval; // Buffer for HTTP response
map<string, string> query; // We store the query
map<int, string> statuses = map_list_of (200, "OK") (400, "Bad request") (404, "Not found") (405, "Unsupported") (500, "Internal error");

int WebServer::parseQueryString(string querystring) {
    vector<string> split, split2;
    boost::split(split, querystring, boost::is_any_of("&"));
    
    // for(map<string, string>::iterator it = query.begin(); it != query.end(); ++it)
    //     std::cout << it->first << " :: " << it->second << std::endl;
    
    query.clear();
    for(int i=0; i< split.size(); i++) {
        if(split.at(i) != "" && split.at(i) != "%22%22") { //TODO Horrible fix: check Seta.html webpage and JS
            //cout << "[" << i << "] " << split.at(i) << endl;
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

char * WebServer::buildResponse(int status, char * res, char * contentType) {
    retval = "HTTP/1.1 " + to_string(status) + " " + statuses[status] +"\n";
    // These two headers are old and deprected on modern browsers, e.g., Chrome
    //retval.append("Cache-Control: no-cache\n");
    //retval.append("Pragma: no-cache\n");
    retval.append("Access-Control-Allow-Origin: *\n");
    retval.append("Access-Control-Allow-Headers: *\n");
    retval.append("Access-Control-Allow-Credentials: true\n");
    if(contentType != NULL) {
        retval.append("Content-Type: ");
        retval.append(contentType);
        retval.append("\n");
    }

    if(res == NULL)
        retval.append("Content-Length: 0\n");
    else
        retval.append("Content-Length: " + to_string (strlen(res)) + "\n");

    if(res != NULL){
        retval.append("\n");
        retval.append(res);
    }

    return (char *) retval.c_str();
}

string json;

char* WebServer::handleBus(string s) {
    //cout << "handleBus(\"" << s << "\")" << endl;

//    return buildResponse(200, "Hello World!", "text/plain");
        
    if(int err = parseQueryString(s))
        return buildResponse(err);

    if(query.size() == 0 || query["id"] == "") {
        cout << "ERROR. 'id' param not specified" << endl;
        return buildResponse(400);
    }

    int idx = atoi(query["id"].c_str());
    idx += 10000;
    //MasaMessage *m = new MasaMessage;
    //m->cam_idx = stoi(query["id"]);
    //m->t_stamp_ms = 1122334455;
    //m->num_objects = rand()%30+1;
    
    
    if(_messages.count(idx) <= 0)
      return buildResponse(200, "{ }", (char *) "application/json");

    MasaMessage *m = _messages[idx];
    if(m->cam_idx > 10000)
      m->cam_idx -= 10000;
    
    //TODO PB use a real lib for JSON
    auto search = _busSizes.find(m->cam_idx);
    int capacity = 100;
    if (search != _busSizes.end())
        capacity = search -> second;
    int fill_percentage = m->num_objects * 100 / capacity;

    //cout << "capacity is " << capacity << " num_objects is " << m->num_objects << " percentage is " << fill_percentage << endl;

    json = "{\n";
    json.append("\t\"cam_idx\" : " + to_string (m->cam_idx) + ",\n");
    json.append("\t\"t_stamp_ms\" : " + to_string (m->t_stamp_ms) + ",\n");
    json.append("\t\"num_objects\" : " + to_string (m->num_objects) + ",\n");
    json.append("\t\"fill_percentage\" : " + to_string (fill_percentage) + "\n");
    json.append("}");

    //delete m;

    return buildResponse(200, (char *)json.c_str(), (char *) "application/json");
}

//TODO PB create a real "HttpUtils" class, or use some kind of Http lib
char* WebServer::handleOptions() {
    retval = "HTTP/1.1 200 OK\n";
    retval.append("Content-Length: 0\n");
    retval.append("Connection: keep-alive\n");
    retval.append("Access-Control-Allow-Origin: *\n");
    retval.append("Access-Control-Allow-Headers: *\n");
    retval.append("Access-Control-Allow-Methods: GET, OPTIONS\n");
    retval.append("Access-Control-Allow-Credentials: true\n");
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
            
        pfunc f = _funcMap[strs.at(0)];
        if(f == NULL) {
            cout << "Handler for endpoint '" << strs.at(0) << "' not found" << endl;
            return buildResponse(404);
        }

        return (this->*f)(querystring);

    }
    else
        return buildResponse(405);
}

void WebServer::OnMessageReceived(MasaMessage *mm) {
    MasaMessage * newm = new MasaMessage;
    newm->cam_idx = mm ->cam_idx;
    newm->t_stamp_ms = mm->t_stamp_ms;
    newm->num_objects = mm->num_objects;

    //cout << " There are " << _messages.count(mm->cam_idx) << " messages from IDX " << mm->cam_idx << endl;
    cout << " There are " << mm->num_objects << " people on bus from IDX " << mm->cam_idx << endl;

    // The old message will be destroyed, so create a copy
    if(_messages.count(mm->cam_idx) > 0) {
      _messages.erase(mm->cam_idx);
      //cout << "There are " << mm->num_objects << " people previously onboard" << endl;
    }
    _messages.insert(std::pair<int, MasaMessage*>(mm->cam_idx, newm));

}

}
