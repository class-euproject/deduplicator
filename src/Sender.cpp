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
    delete lw;
    delete comm;
}

void Sender::start() {
    if (pthread_create(&writerThread, NULL, (THREADFUNCPTR) &Sender::send, this)) 
        perror("could not create thread");
}

void Sender::end() {
    pthread_join(writerThread, NULL);
}

void * Sender::send(void *n) {
    std::vector<MasaMessage> input_messages; //but it must contain only one message.
    MasaMessage send_message;

    while(gRun){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        input_messages = this->cm->getMessages();
        std::cout<<"send dim reading list: "<<input_messages.size()<<std::endl;
        if(input_messages.size() == 0)
            continue;        // no received messages

        if(input_messages.size() > 1){
            send_message = input_messages[input_messages.size()-1];
            std::cout<<"send: too many messages";
        }
        else
            send_message = input_messages[0];


        this->lw->write(send_message);
        
        for (int i = 0; i < comm->size(); i++)
                comm->at(i).send_message(&send_message, this->portList[i]);
        input_messages.clear();
    }
    return (void *)NULL;
}