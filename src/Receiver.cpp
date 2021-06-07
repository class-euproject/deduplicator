#include "Receiver.h"

namespace fog {

Receiver::Receiver(ClassAggregatorMessage &sharedMessage,
                   int port_, int bsc_reciving_port_, bool logWriterFlag) {
    port = port_;
    cm = &sharedMessage;
    lw_flag = logWriterFlag;
    lw = new LogWriter("../demo/data/class_edge_log/");
    comm = new Communicator<MasaMessage>(SOCK_DGRAM);
    bsc_reciving_port = bsc_reciving_port_;
    
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
    Communicator<MasaMessage> bsc_forwarder(SOCK_DGRAM);
    bsc_forwarder.open_client_socket("127.0.0.1", this->bsc_reciving_port);
    //std::cout << "Bsc socket opened at port " << this->bsc_reciving_port << std::endl;
    fog::Profiler prof("Receiver    ");
    while (this->comm->receive_message(this->socketDesc, m) == 0) {
        prof.tick("total time");
        prof.tick("insert message");
        if((*m).cam_idx == 20 or (*m).cam_idx == 21 or (*m).cam_idx == 30 or (*m).cam_idx == 31 or (*m).cam_idx == 40){
            /*auto mess = MasaMessage();
            mess.cam_idx = (*m).cam_idx;
            mess.lights = (*m).lights;
            mess.num_objects = (*m).num_objects;
            mess.objects = (*m).objects;
            mess.t_stamp_ms = (*m).t_stamp_ms;*/
            bsc_forwarder.send_message(m, this->bsc_reciving_port);
        }
        
        //std::cout << (*m).cam_idx << " " << (*m).t_stamp_ms << " " << (*m).objects.size() << std::endl;

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