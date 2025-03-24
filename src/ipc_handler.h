#ifndef IPC_HANDLER_H
#define IPC_HANDLER_H
#include "decoder.h"
#include <string>
#include <memory>

#define SOCKET_PATH     "./"

enum TraceDataType {
    TRACE_DATA_START_TRACE = 0,
    TRACE_DATA_BASIC_BLOCK = 1,
    TRACE_DATA_CODE = 2,
    TRACE_DATA_LOAD_STORE = 3
};

class IPCHandler {
private:
    int thread_id;
    std::string socketPath = "";
    int serverFd = -1;
    int clientFd = -1;

    size_t readExactBytes(int fd, void *buffer, size_t size);

    void *readData(bool *endOfThread, enum TraceDataType dataType, void *buffer = nullptr);
public:
    IPCHandler(THREADID tid);

    ~IPCHandler();

    void waitAccept();

    struct FrontendTrace *receiveTrace();
};

#endif
