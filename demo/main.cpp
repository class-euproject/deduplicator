#include <iostream>
#include "utils.h"
#include "configuration.h"
#include "ClassAggregatorMessage.h"
#include "Receiver.h"
#include "Sender.h"
#include "Deduplicator.h"
#include "Aggregator.h"

int main(int argc, char **argv) {
    ClassAggregatorMessage received_messages;       // Receiver fills this message, Deduplicator deduplicates its objects
    ClassAggregatorMessage deduplicated_messages;   // Deduplicator creates this message without duplicator, Aggregator adds some info
    ClassAggregatorMessage aggregated_messages;     // Aggregator creates this message, Sender sends it to all dest.

    // read yaml parameters file
    Parameters_t param;
    if(!readParameters(argc, argv, &param)) {
        exit(EXIT_SUCCESS);         // help
    }

    gRun = true;

    // start the viewer
    AggregatorViewer v(param.pngFile);
    v.setWindowName("Map");
    v.setBackground(tk::gui::color::DARK_GRAY);
    v.initOnThread();
    
    // TODO: iterate on ports:
    if (param.inputPortList.size() > 1)
        std::cerr<<"many input ports: no supported. Use only the first...\n";
    Receiver r(received_messages, param.inputPortList[0]);
    r.start();

    Deduplicator d(received_messages, deduplicated_messages, param.tifFile, v, param.visualization);
    d.start();

    Aggregator a(deduplicated_messages, aggregated_messages, v, param.visualization);
    a.start();

    Sender s(aggregated_messages, param.outputIpList, param.outputPortList);
    s.start();

    v.joinThread();
    r.end();
    d.end();
    a.end();
    s.end();
    return EXIT_SUCCESS;
}