#include "Sender.h"

Sender::Sender(ClassAggregatorMessage &sharedMessage,
                   std::vector<std::string> ipList_,
                   std::vector<int> portList_) {
    assert(ipList_.size() == portList_.size());
    
    numComm = portList_.size();
    cm = &sharedMessage;
    ipList = ipList_;
    portList = portList_;

    lw = new LogWriter("./demo/data/class_fog_log/");

    comm = new std::vector<Communicator<MasaMessage>>(SOCK_DGRAM);
    for(int i = 0; i<numComm; i++) {
        Communicator<MasaMessage> p;
        comm->push_back(p);
        comm->at(i).open_client_socket((char *)ipList[i].c_str(), portList[i]);
        socketDescList.push_back(comm->at(i).get_socket());
    }
}

Sender::~Sender() {
    free(lw);
    free(comm);
}

void Sender::start() {
    if (pthread_create(&writerThread, NULL, (THREADFUNCPTR) &Sender::send, this)) 
        perror("could not create thread");
}

void Sender::end() {
    pthread_join(writerThread, NULL);
}

void * Sender::send(void *n) {
    std::vector<MasaMessage> m; //but m must be contain only one message.

    while(true){
        std::cout<<"get m\n";
        m = this->cm->getMessages();
        if(m.size() == 0)
            continue;        // no received messages

        if(m.size() != 1) 
            std::cout<<"Warning. There are more than one aggregate message.\n";
        std::cout<<"write m\n";
        this->lw->write(m.at(0));
        
        for (int i = 0; i < comm->size(); i++)
                comm->at(i).send_message(&m.at(0), this->portList[i]);
        m.clear();
        break;
    }
    return (void *)NULL;
}