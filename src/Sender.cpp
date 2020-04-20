#include "Sender.h"

Sender::Sender(ClassAggregatorMessage &sharedMessage,
                   std::vector<std::string> ipList,
                   std::vector<int> portList) {
    assert(ipList.size() == portList.size());
    
    num_comm = portList.size();
    cm = &sharedMessage;
    ip_list = ipList;
    port_list = portList;

    lw = new LogWriter("./demo/data/class_fog_log/");

    comm = new std::vector<Communicator<MasaMessage>>(SOCK_DGRAM);
    for(int i = 0; i<num_comm; i++) {
        Communicator<MasaMessage> p;
        comm->push_back(p);
        comm->at(i).open_client_socket((char *)ip_list[i].c_str(), port_list[i]);
        socket_desc_list.push_back(comm->at(i).get_socket());
    }
}

Sender::~Sender() {
    free(lw);
    free(comm);
}

void Sender::start() {
    if (pthread_create(&writer_thread, NULL, (THREADFUNCPTR) &Sender::send, this)) 
        perror("could not create thread");
}

void Sender::end() {
    pthread_join(writer_thread, NULL);
}

void * Sender::send(void *n) {
    std::vector<MasaMessage> m; //but m must be contain only one message.

    while(true){
        std::cout<<"get m\n";
        m = this->cm->getMessages();
        if(m.size() != 1) 
            std::cout<<"Warning. There are more than one aggregate message.\n";
        std::cout<<"write m\n";
        this->lw->write(m.at(0));
        
        for (int i = 0; i < comm->size(); i++)
                comm->at(i).send_message(&m.at(0), this->port_list[i]);
        m.clear();
        break;
    }
    return (void *)NULL;
}





