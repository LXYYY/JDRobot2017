//
// Created by root on 3/2/17.
//

#include "pthread.h"
#include "Threads.h"

void *CommunicateThread(void *arg) {
    communicator.connect();
    while (1) {
        for (size_t i = 0; i < communicator.PtsToSend.size(); i++) {
            communicator.sendFrame(Communication::PointID,i);
            communicator.sendFrame(Communication::PointX,communicator.PtsToSend.at(i).second.x);
            communicator.sendFrame(Communication::PointY,communicator.PtsToSend.at(i).second.y);
            communicator.sendFrame(Communication::PointZ,communicator.PtsToSend.at(i).second.z);
        }
    }
}

bool createThread() {
    pthread_t id_1, id_2;
    int ret;
    ret = pthread_create(&id_1, NULL, CommunicateThread, NULL);
    return ret;
}