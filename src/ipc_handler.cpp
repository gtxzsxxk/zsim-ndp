#include "ipc_handler.h"
#include "core.h"
#include <cstddef>
#include <memory>
#include <sstream>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

IPCHandler::IPCHandler(THREADID tid) {
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

void *IPCHandler::readData(enum TraceDataType dataType, void *buffer) {
    uint32_t expectSize;
    if (readExactBytes(clientFd, &expectSize, sizeof(expectSize)) <= 0) {
        panic("Cannot receive data size");
    }
    uint32_t actualDataType;
    if (readExactBytes(clientFd, &actualDataType, sizeof(actualDataType)) <= 0) {
        panic("Cannot receive data type");
    }
    if ((uint32_t) dataType != actualDataType) {
        panic("Cannot resolve packet");
    }
    if (!buffer) {
        buffer = new char[expectSize];
    }
    if (readExactBytes(clientFd, buffer, expectSize) <= 0) {
        panic("Cannot resolve packet body");
    }
    return buffer;
}

/*
 * packet format: size(word) type(word) data
 */
 std::unique_ptr<struct FrontendTrace> IPCHandler::receiveTrace() {
    auto frontendTrace = std::unique_ptr<struct FrontendTrace>((struct FrontendTrace *)readData(TRACE_DATA_START_TRACE));
    frontendTrace->blocks = new struct BasicBlock[frontendTrace->count];
    for (size_t i = 0; i < frontendTrace->count; i++) {
        readData(TRACE_DATA_BASIC_BLOCK, (void *) &frontendTrace->blocks[i]);
        /* receive basic block data */
        /* receive code */
        frontendTrace->blocks[i].code = reinterpret_cast<uint8_t *>(readData(TRACE_DATA_CODE));
        /* receive load and store */
        auto instCount = frontendTrace->blocks[i].getInstructionCount();
        frontendTrace->blocks[i].loadStore = new struct BasicBlockLoadStore[instCount];
        for (size_t j = 0; j < instCount; j++) {
            readData(TRACE_DATA_LOAD_STORE, &frontendTrace->blocks[i].loadStore[j]);
            auto next = &frontendTrace->blocks[i].loadStore[j].next;
            while (*next) {
                *next = reinterpret_cast<struct BasicBlockLoadStore *>(readData(TRACE_DATA_LOAD_STORE));
                next = &((*next)->next);
            }
        }
    }
    return frontendTrace;
}
