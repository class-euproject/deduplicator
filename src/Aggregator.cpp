#include "Aggregator.h"

Aggregator::Aggregator(ClassAggregatorMessage &inputSharedMessage, 
                       ClassAggregatorMessage &outputSharedMessage,
                       AggregatorViewer &v,
                       bool visual) {
    inCm = &inputSharedMessage;
    outCm = &outputSharedMessage;
    viewer = &v;
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
    while(gRun){
        // TODO: introduce some delay? (active wait) 
        input_messages = this->inCm->getMessages();
        if(input_messages.size() == 0)
            continue;        // no received messages

        if(input_messages.size() > 1){
            aggregate_message = input_messages[input_messages.size()-1];
            std::cout<<"agg: too many messages";
        }
        else
            aggregate_message = input_messages[0];
        
        // TODO: add some other information

        // if(this->show)
        //     show something

        std::cout<<"insert m\n";
        this->outCm->insertMessage(aggregate_message);
    }
    return (void *)NULL;
}