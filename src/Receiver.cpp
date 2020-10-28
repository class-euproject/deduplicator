#include "Receiver.h"

namespace fog {

Receiver::Receiver(ClassAggregatorMessage &sharedMessage,
                   int port_, bool logWriterFlag) {
    port = port_;
    cm = &sharedMessage;
    lw_flag = logWriterFlag;
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
    fog::Profiler prof("Receiver    ");
    while (this->comm->receive_message(this->socketDesc, m) == 0) {
        prof.tick("total time");
        prof.tick("insert message");
        this->cm->insertMessage(*m);
        prof.tock("insert message");
        if(lw_flag) {
            prof.tick("write edge");
            this->lw->write(*m);
            prof.tock("write edge");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        prof.tock("total time");
        //prof.printStats();
    }
    delete m;
    return (void *)NULL;
}
}