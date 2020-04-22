#include "Receiver.h"

Receiver::Receiver(ClassAggregatorMessage &sharedMessage,
                   int port_) {
    port = port_;
    cm = &sharedMessage;
    lw = new LogWriter("./demo/data/class_fog_log/");
    comm = new Communicator<MasaMessage>(SOCK_DGRAM);
    
    comm->open_server_socket(port);
    socketDesc = comm->get_socket();
    if (socketDesc == -1)
        perror("error in socket");
}

Receiver::~Receiver() {
    free(lw);
    free(comm);
}

void Receiver::start() {
    if (pthread_create(&snifferThread, NULL, (THREADFUNCPTR) &Receiver::receive, this)) 
        perror("could not create thread");
}

void Receiver::end() {
    pthread_join(snifferThread, NULL);
}

void * Receiver::receive(void *n) {
    MasaMessage *m;
    while (this->comm->receive_message(this->socketDesc, m) == 0) {
        std::cout<<"insert m\n";
        this->cm->insertMessage(*m);
        std::cout<<"write m\n";
        this->lw->write(*m);
        break;
    }
    free(m);
    return (void *)NULL;
}