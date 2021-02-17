#include "Aggregator.h"

namespace fog {

Aggregator::Aggregator(ClassAggregatorMessage &inputSharedMessage, 
                       ClassAggregatorMessage &outputSharedMessage,
                       bool visual) {
    inCm = &inputSharedMessage;
    outCm = &outputSharedMessage;
    show = visual;
}

Aggregator::~Aggregator() {
}

void Aggregator::start() {
    if (pthread_create(&aggregatorThread, NULL, (THREADFUNCPTR) &Aggregator::aggregate, this)) 
        perror("could not create thread");
}

void Aggregator::end() {
    pthread_join(aggregatorThread, NULL);
}

void * Aggregator::aggregate(void *n) {
    std::vector<MasaMessage> input_messages; 
    MasaMessage aggregate_message;
    fog::Profiler prof("Aggregator  ");
    while(gRun){
        prof.tick("total time");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        prof.tick("get messages");
        input_messages = this->inCm->getMessages();
        prof.tock("get messages");
        // std::cout<<"agg dim reading list: "<<input_messages.size()<<std::endl;
        if(input_messages.size() == 0) {
            prof.tock("total time");
            continue;        // no received messages
        }
            
        if(input_messages.size() > 1){
            aggregate_message = input_messages[input_messages.size()-1];
            // std::cout<<"agg: too many messages";
        }
        else
            aggregate_message = input_messages[0];
        
        // TODO: add some other information

        // if(this->show)
        //     show something
        prof.tick("insert message ");
        this->outCm->insertMessage(aggregate_message);
        input_messages.clear();
        prof.tock("insert message ");
        prof.tock("total time");
        //prof.printStats();
    }
    return (void *)NULL;
}
}