#include "ipc_handler.h"
#include "core.h"
#include <cstddef>
#include <memory>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#define panicAndEnd(args...) \
{ \
    fprintf(logFdErr, "%sPanic on %s:%d: ", logHeader, __FILE__, __LINE__); \
    fprintf(logFdErr, args); \
    fprintf(logFdErr, "\n"); \
    fflush(logFdErr); \
    *endOfThread = true; \
    return nullptr; \
}

IPCHandler::IPCHandler(THREADID tid): thread_id(tid) {
    std::stringstream ss;
    ss << SOCKET_PATH << "trace_in_uds_" << tid;
    socketPath = ss.str();

    serverFd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (serverFd < 0) {
        panic("Failed to create Unix domain socket");
    }

    sockaddr_un serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sun_family = AF_UNIX;
    strncpy(serverAddr.sun_path, socketPath.c_str(), sizeof(serverAddr.sun_path) - 1);

    /* delete existing domain socket file */
    unlink(socketPath.c_str());

    if (bind(serverFd, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        panic("Bind failed");
        close(serverFd);
        return;
    }

    if (listen(serverFd, 1) < 0) {
        panic("Listen failed");
        close(serverFd);
    }
}

IPCHandler::~IPCHandler() {
    if (clientFd != -1) {
        close(clientFd);
    }
    if (serverFd != -1) {
        close(serverFd);
    }
    unlink(socketPath.c_str());
}

void IPCHandler::waitAccept() {
    clientFd = accept(serverFd, nullptr, nullptr);
    if (clientFd < 0) {
        panic("Accept failed");
    }
}

size_t IPCHandler::readExactBytes(int fd, void *buffer, size_t size) {
    size_t recv = 0;
    while (recv < size) {
        auto result = read(fd, (char*)buffer + recv, size - recv);
        if (result <= 0) return result;
        recv += result;
    }
    return recv;
}

void *IPCHandler::readData(bool *endOfThread, enum TraceDataType dataType, void *buffer) {
    uint32_t expectSize;
    if (readExactBytes(clientFd, &expectSize, sizeof(expectSize)) <= 0) {
        panicAndEnd("Cannot receive data size");
    }
    uint32_t actualDataType;
    if (readExactBytes(clientFd, &actualDataType, sizeof(actualDataType)) <= 0) {
        panicAndEnd("Cannot receive data type");
    }
    if ((uint32_t) dataType != actualDataType) {
        panicAndEnd("Cannot resolve packet");
    }
    char *buf__ = nullptr;
    if (!buffer) {
        buffer = new char[expectSize];
        buf__ = (char *)buffer;
    }
    if (readExactBytes(clientFd, buffer, expectSize) <= 0) {
        if (buf__) {
            delete[] buf__;
        }
        panicAndEnd("Cannot resolve packet body");
    }
    return buffer;
}

/*
 * packet format: size(word) type(word) data
 */
struct FrontendTrace *IPCHandler::receiveTrace() {
    bool needEnd = false;
    auto frontendTrace = (struct FrontendTrace *)readData(&needEnd, TRACE_DATA_START_TRACE);
    if (needEnd) {
        return nullptr;
    }
    frontendTrace->blocks = new struct BasicBlock[frontendTrace->count];
    for (size_t i = 0; i < frontendTrace->count; i++) {
        readData(&needEnd, TRACE_DATA_BASIC_BLOCK, (void *) &frontendTrace->blocks[i]);
        if (needEnd) {
            return nullptr;
        }
        /* receive basic block data */
        /* receive code */
        frontendTrace->blocks[i].code = reinterpret_cast<uint8_t *>(readData(&needEnd, TRACE_DATA_CODE));
        if (needEnd) {
            return nullptr;
        }
        /* receive load and store */
        frontendTrace->blocks[i].loadStore = new struct BasicBlockLoadStore[frontendTrace->blocks[i].loadStores];
        for (size_t j = 0; j < frontendTrace->blocks[i].loadStores; j++) {
            readData(&needEnd, TRACE_DATA_LOAD_STORE, &frontendTrace->blocks[i].loadStore[j]);
            if (needEnd) {
                return nullptr;
            }
            auto next = &frontendTrace->blocks[i].loadStore[j].next;
            while (*next) {
                *next = reinterpret_cast<struct BasicBlockLoadStore *>(readData(&needEnd, TRACE_DATA_LOAD_STORE));
                if (needEnd) {
                    return nullptr;
                }
                next = &((*next)->next);
            }
        }
    }
    return frontendTrace;
}
