#include "Communication.h"


Communication::Communication() {
}


Communication::~Communication() {
}

bool Communication::setFrame(unsigned char tDataType, float tData) {
    frame[Head1] = HEAD1;
    frame[Head2] = HEAD2;
    frame[Tail] = TAIL;
    frame[dataType] = tDataType;
    BitConverter.value = tData;
    frame[data0] = BitConverter.byte[0];
    frame[data1] = BitConverter.byte[1];
    frame[data2] = BitConverter.byte[2];
    frame[data3] = BitConverter.byte[3];
    frame[sumCheck] = frame[data2] + frame[data3];
    return true;
}

void Communication::FrameAnalysis(unsigned char buff[], int buff_len) {
    unsigned char frame_fifo[FrameLen];
    for (int i = 0; i < buff_len; i++) {
        //FIFO??
        for (int j = 0; j < FrameLen - 1; j++) {
            frame_fifo[j] = frame_fifo[j + 1];
        }
        frame_fifo[FrameLen - 1] = buff[i];

        if (frame_fifo[Head1] == HEAD1
            && frame_fifo[Head2] == HEAD2
            && frame_fifo[Tail] == TAIL) {
            unsigned char id = frame_fifo[dataType];
            BitConverter.byte[0] = frame_fifo[data0];
            BitConverter.byte[1] = frame_fifo[data1];
            BitConverter.byte[2] = frame_fifo[data2];
            BitConverter.byte[3] = frame_fifo[data3];

            if (frame_fifo[sumCheck] == (frame_fifo[data2] + frame_fifo[data3]))      ////sumcheck
                if (id <= nDataTypes) {
                    MsgFromMach[id] = BitConverter.value;
                }
        }
    }
}

bool Communication::sendMsg() {
    socket.sendMsg(frame);
    return false;
}

bool Communication::sendFrame(unsigned char dataType, float data) {
    setFrame(dataType, data);
    if (socket.sendMsg(frame)) {
        return true;
    } ///发送
    return false;
}

bool Communication::connect(void) {
    if(!socket.connetServer()){
        perror("commucation connect failed");
        return false;
    }
    return true;
}
