#include "Receiver.h"

namespace fog {

Receiver::Receiver(ClassAggregatorMessage &sharedMessage,
                   int port_) {
    port = port_;
    cm = &sharedMessage;
    lw = new LogWriter("../demo/data/class_edge_log/");
    comm = new Communicator<MasaMessage>(SOCK_DGRAM);
    
    comm->open_server_socket(port);
    socketDesc = comm->get_socket();
    if (socketDesc == -1)
        perror("error in socket");
}

Receiver::~Receiver() {
    delete lw;
    delete comm;
}

void Receiver::start() {
    if (pthread_create(&snifferThread, NULL, (THREADFUNCPTR) &Receiver::receive, this)) 
        perror("could not create thread");
}

void Receiver::end() {
    pthread_join(snifferThread, NULL);
}

void * Receiver::receive(void *n) {
    MasaMessage *m = new MasaMessage();
    std::cout<<"started\n";
    while (this->comm->receive_message(this->socketDesc, m) == 0) {
        this->cm->insertMessage(*m);
        this->lw->write(*m);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    delete m;
    return (void *)NULL;
}
}