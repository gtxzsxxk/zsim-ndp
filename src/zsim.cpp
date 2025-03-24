/** $glic$
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
 * Copyright (C) 2011 Google Inc.
 *
 * This file is part of zsim.
 *
 * zsim is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, version 2.
 *
 * If you use this software in your research, we request that you reference
 * the zsim paper ("ZSim: Fast and Accurate Microarchitectural Simulation of
 * Thousand-Core Systems", Sanchez and Kozyrakis, ISCA-40, June 2013) as the
 * source of the simulator in any publications that use this software, and that
 * you send us a citation of your work.
 *
 * zsim is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

/* The Pin-facing part of the simulator */

#include "zsim.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <signal.h>
#include <dlfcn.h>
#include <execinfo.h>
#include <fstream>
#include <iostream>
#include <libunwind.h>
#include <sched.h>
#include <sstream>
#include <string>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <thread>
#include <queue>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <unistd.h>
#include "access_tracing.h"
#include "config.h"
#include "constants.h"
#include "contention_sim.h"
#include "core.h"
#include "decoder.h"
#include "cpuenum.h"
#include "cpuid.h"
#include "debug_zsim.h"
#include "event_queue.h"
#include "galloc.h"
#include "init.h"
#include "ipc_handler.h"
#include "log.h"
#include "process_tree.h"
#include "profile_stats.h"
#include "scheduler.h"
#include "stats.h"

using namespace std;

//#include <signal.h> //can't include this, conflicts with PIN's

/* Command-line switches (used to pass info from harness that cannot be passed through the config file, most config is file-based) */


/* Global Variables */

GlobSimInfo* zinfo;

/* Per-process variables */

uint32_t procIdx;
uint32_t lineBits; //process-local for performance, but logically global
uint32_t pageBits; //process-local for performance, but logically global
Address procMask;

static ProcessTreeNode* procTreeNode;

//tid to cid translation
#define INVALID_CID ((uint32_t)-1)
#define UNINITIALIZED_CID ((uint32_t)-2) //Value set at initialization

static uint32_t cids[MAX_THREADS];

// Per TID core pointers (TODO: phase out cid/tid state --- this is enough)
Core* cores[MAX_THREADS];

static inline void clearCid(uint32_t tid) {
    assert(tid < MAX_THREADS);
    assert(cids[tid] != INVALID_CID);
    cids[tid] = INVALID_CID;
    cores[tid] = nullptr;
}

static inline void setCid(uint32_t tid, uint32_t cid) {
    assert(tid < MAX_THREADS);
    assert(cids[tid] == INVALID_CID);
    assert(cid < zinfo->numCores);
    cids[tid] = cid;
    cores[tid] = zinfo->cores[cid];
}

uint32_t getCid(uint32_t tid) {
    //assert(tid < MAX_THREADS); //these assertions are fine, but getCid is called everywhere, so they are expensive!
    uint32_t cid = cids[tid];
    //assert(cid != INVALID_CID);
    return cid;
}

// Internal function declarations
void EnterFastForward();
void ExitFastForward();

void SimThreadStart(THREADID tid);
void SimThreadFini(THREADID tid);
void SimEnd();

void HandleMagicOp(THREADID tid, ADDRINT op);

void VdsoInstrument(INS ins);
void FFThread(void* arg);

/* Indirect analysis calls to work around PIN's synchronization
 *
 * NOTE(dsm): Be extremely careful when modifying this code. It is simple, but
 * it runs VERY frequently.  For example, with 24-byte structs on a fairly
 * unoptimized L1 cache, this code introduced a 4% overhead, down to 2% with
 * 32-byte structs. Also, be aware that a miss or unpredictable indirect jump
 * is about the worst kind of pain you can inflict on an ooo core, so ensure
 * that 1) there's no false sharing, and 2) these pointers are modified
 * sparingly.
 */

InstrFuncPtrs fPtrs[MAX_THREADS] ATTR_LINE_ALIGNED; //minimize false sharing

void IndirectLoadSingle(THREADID tid, ADDRINT addr) {
    fPtrs[tid].loadPtr(tid, addr);
}

void IndirectStoreSingle(THREADID tid, ADDRINT addr) {
    fPtrs[tid].storePtr(tid, addr);
}

void IndirectBasicBlock(THREADID tid, ADDRINT bblAddr, BblInfo* bblInfo) {
    fPtrs[tid].bblPtr(tid, bblAddr, bblInfo);
}

void IndirectRecordBranch(THREADID tid, ADDRINT branchPc, BOOL taken, ADDRINT takenNpc, ADDRINT notTakenNpc) {
    fPtrs[tid].branchPtr(tid, branchPc, taken, takenNpc, notTakenNpc);
}

void IndirectPredLoadSingle(THREADID tid, ADDRINT addr, BOOL pred) {
    fPtrs[tid].predLoadPtr(tid, addr, pred);
}

void IndirectPredStoreSingle(THREADID tid, ADDRINT addr, BOOL pred) {
    fPtrs[tid].predStorePtr(tid, addr, pred);
}


//Non-simulation variants of analysis functions

// Join variants: Call join on the next instrumentation poin and return to analysis code
void Join(uint32_t tid) {
    assert(fPtrs[tid].type == FPTR_JOIN);
    uint32_t cid = zinfo->sched->join(procIdx, tid); //can block
    setCid(tid, cid);

    if (unlikely(zinfo->terminationConditionMet)) {
        info("Caught termination condition on join, exiting");
        zinfo->sched->leave(procIdx, tid, cid);
        SimEnd();
    }

    fPtrs[tid] = cores[tid]->GetFuncPtrs(); //back to normal pointers
}

void JoinAndLoadSingle(THREADID tid, ADDRINT addr) {
    Join(tid);
    fPtrs[tid].loadPtr(tid, addr);
}

void JoinAndStoreSingle(THREADID tid, ADDRINT addr) {
    Join(tid);
    fPtrs[tid].storePtr(tid, addr);
}

void JoinAndBasicBlock(THREADID tid, ADDRINT bblAddr, BblInfo* bblInfo) {
    Join(tid);
    fPtrs[tid].bblPtr(tid, bblAddr, bblInfo);
}

void JoinAndRecordBranch(THREADID tid, ADDRINT branchPc, BOOL taken, ADDRINT takenNpc, ADDRINT notTakenNpc) {
    Join(tid);
    fPtrs[tid].branchPtr(tid, branchPc, taken, takenNpc, notTakenNpc);
}

void JoinAndPredLoadSingle(THREADID tid, ADDRINT addr, BOOL pred) {
    Join(tid);
    fPtrs[tid].predLoadPtr(tid, addr, pred);
}

void JoinAndPredStoreSingle(THREADID tid, ADDRINT addr, BOOL pred) {
    Join(tid);
    fPtrs[tid].predStorePtr(tid, addr, pred);
}

// NOP variants: Do nothing
void NOPLoadStoreSingle(THREADID tid, ADDRINT addr) {}
void NOPBasicBlock(THREADID tid, ADDRINT bblAddr, BblInfo* bblInfo) {}
void NOPRecordBranch(THREADID tid, ADDRINT addr, BOOL taken, ADDRINT takenNpc, ADDRINT notTakenNpc) {}
void NOPPredLoadStoreSingle(THREADID tid, ADDRINT addr, BOOL pred) {}

// FF is basically NOP except for basic blocks
void FFBasicBlock(THREADID tid, ADDRINT bblAddr, BblInfo* bblInfo) {
    if (unlikely(!procTreeNode->isInFastForward())) {
        SimThreadStart(tid);
    }
}

// FFI is instruction-based fast-forwarding
/* FFI works as follows: when in fast-forward, we install a special FF BBL func
 * ptr that counts instructions and checks whether we have reached the switch
 * point. Then, it exits FF, and queues an event that counts the instructions
 * where the app should be scheduled. That event cannot access any local state,
 * so when it hits the limit, it just makes the process enter FF. On that
 * entry, we install a special handler that advances to the next FFI point and
 * installs the normal FFI handlers (pretty much like joins work).
 *
 * REQUIREMENTS: Single-threaded during FF (non-FF can be MT)
 */

//TODO (dsm): Went for quick, dirty and contained here. This could use a cleanup.

// FFI state
static bool ffiEnabled;
static uint32_t ffiPoint;
static uint64_t ffiInstrsDone;
static uint64_t ffiInstrsLimit;
static bool ffiNFF;

//Track the non-FF instructions executed at the beginning of this and last interval.
//Can only be updated at ends of phase, by the NFF tracking event.
static uint64_t* ffiFFStartInstrs; //hack, needs to be a pointer, written to outside this process
static uint64_t* ffiPrevFFStartInstrs;

static const InstrFuncPtrs& GetFFPtrs();

void FFITrackNFFInterval() {
    assert(!procTreeNode->isInFastForward());
    assert(ffiInstrsDone < ffiInstrsLimit); //unless you have ~10-instr FFWds, this does not happen

    //Queue up an event to detect and end FF
    //Note vars are captured, so these lambdas can be called from any process
    uint64_t startInstrs = *ffiFFStartInstrs;
    uint32_t p = procIdx;
    uint64_t* _ffiFFStartInstrs = ffiFFStartInstrs;
    uint64_t* _ffiPrevFFStartInstrs = ffiPrevFFStartInstrs;
    auto ffiGet = [p, startInstrs]() { return zinfo->processStats->getProcessInstrs(p) - startInstrs; };
    auto ffiFire = [p, _ffiFFStartInstrs, _ffiPrevFFStartInstrs]() {
        info("FFI: Entering fast-forward for process %d", p);
        /* Note this is sufficient due to the lack of reinstruments on FF, and this way we do not need to touch global state */
        futex_lock(&zinfo->ffLock);
        assert(!zinfo->procArray[p]->isInFastForward());
        zinfo->procArray[p]->enterFastForward();
        futex_unlock(&zinfo->ffLock);
        *_ffiPrevFFStartInstrs = *_ffiFFStartInstrs;
        *_ffiFFStartInstrs = zinfo->processStats->getProcessInstrs(p);
    };
    zinfo->eventQueue->insert(makeAdaptiveEvent(ffiGet, ffiFire, 0, ffiInstrsLimit - ffiInstrsDone, MAX_IPC*zinfo->phaseLength));

    ffiNFF = true;
}

// Called on process start
void FFIInit() {
    const g_vector<uint64_t>& ffiPoints = procTreeNode->getFFIPoints();
    if (!ffiPoints.empty()) {
        if (zinfo->ffReinstrument) panic("FFI and reinstrumenting on FF switches are incompatible");
        ffiEnabled = true;
        ffiPoint = 0;
        ffiInstrsDone = 0;
        ffiInstrsLimit = ffiPoints[0];

        ffiFFStartInstrs = gm_calloc<uint64_t>(1);
        ffiPrevFFStartInstrs = gm_calloc<uint64_t>(1);
        ffiNFF = false;
        info("FFI mode initialized, %ld ffiPoints", ffiPoints.size());
        if (!procTreeNode->isInFastForward()) FFITrackNFFInterval();
    } else {
        ffiEnabled = false;
    }
}

//Set the next ffiPoint, or finish
void FFIAdvance() {
    const g_vector<uint64_t>& ffiPoints = procTreeNode->getFFIPoints();
    ffiPoint++;
    if (ffiPoint >= ffiPoints.size()) {
        info("Last ffiPoint reached, %ld instrs, limit %ld", ffiInstrsDone, ffiInstrsLimit);
        SimEnd();
    } else {
        info("ffiPoint reached, %ld instrs, limit %ld", ffiInstrsDone, ffiInstrsLimit);
        ffiInstrsLimit += ffiPoints[ffiPoint];
    }
}

void FFIBasicBlock(THREADID tid, ADDRINT bblAddr, BblInfo* bblInfo) {
    ffiInstrsDone += bblInfo->instrs;
    if (unlikely(ffiInstrsDone >= ffiInstrsLimit)) {
        FFIAdvance();
        assert(procTreeNode->isInFastForward());
        futex_lock(&zinfo->ffLock);
        info("FFI: Exiting fast-forward");
        ExitFastForward();
        futex_unlock(&zinfo->ffLock);
        FFITrackNFFInterval();

        SimThreadStart(tid);
    }
}

// One-off, called after we go from NFF to FF
void FFIEntryBasicBlock(THREADID tid, ADDRINT bblAddr, BblInfo* bblInfo) {
    ffiInstrsDone += *ffiFFStartInstrs - *ffiPrevFFStartInstrs; //add all instructions executed in the NFF phase
    FFIAdvance();
    assert(ffiNFF);
    ffiNFF = false;
    fPtrs[tid] = GetFFPtrs();
    FFIBasicBlock(tid, bblAddr, bblInfo);
}

// Non-analysis pointer vars
static const InstrFuncPtrs joinPtrs = {JoinAndLoadSingle, JoinAndStoreSingle, JoinAndBasicBlock, JoinAndRecordBranch, JoinAndPredLoadSingle, JoinAndPredStoreSingle, FPTR_JOIN};
static const InstrFuncPtrs nopPtrs = {NOPLoadStoreSingle, NOPLoadStoreSingle, NOPBasicBlock, NOPRecordBranch, NOPPredLoadStoreSingle, NOPPredLoadStoreSingle, FPTR_NOP};
static const InstrFuncPtrs retryPtrs = {NOPLoadStoreSingle, NOPLoadStoreSingle, NOPBasicBlock, NOPRecordBranch, NOPPredLoadStoreSingle, NOPPredLoadStoreSingle, FPTR_RETRY};
static const InstrFuncPtrs ffPtrs = {NOPLoadStoreSingle, NOPLoadStoreSingle, FFBasicBlock, NOPRecordBranch, NOPPredLoadStoreSingle, NOPPredLoadStoreSingle, FPTR_NOP};

static const InstrFuncPtrs ffiPtrs = {NOPLoadStoreSingle, NOPLoadStoreSingle, FFIBasicBlock, NOPRecordBranch, NOPPredLoadStoreSingle, NOPPredLoadStoreSingle, FPTR_NOP};
static const InstrFuncPtrs ffiEntryPtrs = {NOPLoadStoreSingle, NOPLoadStoreSingle, FFIEntryBasicBlock, NOPRecordBranch, NOPPredLoadStoreSingle, NOPPredLoadStoreSingle, FPTR_NOP};

static const InstrFuncPtrs& GetFFPtrs() {
    return ffiEnabled? (ffiNFF? ffiEntryPtrs : ffiPtrs) : ffPtrs;
}

//Fast-forwarding
void EnterFastForward() {
    assert(!procTreeNode->isInFastForward());
    procTreeNode->enterFastForward();
    __sync_synchronize(); //Make change globally visible

    //Transition to FF; we have the ff lock, this should be safe with end of phase code. This avoids profiling the end of a simulation as bound time
    //NOTE: Does not work well with multiprocess runs
    zinfo->profSimTime->transition(PROF_FF);
}


void ExitFastForward() {
    assert(procTreeNode->isInFastForward());

    procTreeNode->exitFastForward();
    __sync_synchronize(); //make change globally visible
}



//Termination
volatile uint32_t perProcessEndFlag;

void SimEnd();

void CheckForTermination() {
    assert(zinfo->terminationConditionMet == false);
    if (zinfo->maxPhases && zinfo->numPhases >= zinfo->maxPhases) {
        zinfo->terminationConditionMet = true;
        info("Max phases reached (%ld)", zinfo->numPhases);
        return;
    }

    if (zinfo->maxMinInstrs) {
        uint64_t minInstrs = zinfo->cores[0]->getInstrs();
        for (uint32_t i = 1; i < zinfo->numCores; i++) {
            uint64_t coreInstrs = zinfo->cores[i]->getInstrs();
            if (coreInstrs < minInstrs && coreInstrs > 0) {
                minInstrs = coreInstrs;
            }
        }

        if (minInstrs >= zinfo->maxMinInstrs) {
            zinfo->terminationConditionMet = true;
            info("Max min instructions reached (%ld)", minInstrs);
            return;
        }
    }

    if (zinfo->maxTotalInstrs) {
        uint64_t totalInstrs = 0;
        for (uint32_t i = 0; i < zinfo->numCores; i++) {
            totalInstrs += zinfo->cores[i]->getInstrs();
        }

        if (totalInstrs >= zinfo->maxTotalInstrs) {
            zinfo->terminationConditionMet = true;
            info("Max total (aggregate) instructions reached (%ld)", totalInstrs);
            return;
        }
    }

    if (zinfo->maxSimTimeNs) {
        uint64_t simNs = zinfo->profSimTime->count(PROF_BOUND) + zinfo->profSimTime->count(PROF_WEAVE);
        if (simNs >= zinfo->maxSimTimeNs) {
            zinfo->terminationConditionMet = true;
            info("Max simulation time reached (%ld ns)", simNs);
            return;
        }
    }

    if (zinfo->externalTermPending) {
        zinfo->terminationConditionMet = true;
        info("Terminating due to external notification");
        return;
    }
}

/* This is called by the scheduler at the end of a phase. At that point, zinfo->numPhases
 * has not incremented, so it denotes the END of the current phase
 */
void EndOfPhaseActions() {
    zinfo->profSimTime->transition(PROF_WEAVE);
    if (zinfo->globalPauseFlag) {
        info("Simulation entering global pause");
        zinfo->profSimTime->transition(PROF_FF);
        while (zinfo->globalPauseFlag) usleep(20*1000);
        zinfo->profSimTime->transition(PROF_WEAVE);
        info("Global pause DONE");
    }

    // Done before tick() to avoid deadlock in most cases when entering synced ffwd (can we still deadlock with sleeping threads?)
    if (unlikely(zinfo->globalSyncedFFProcs)) {
        info("Simulation paused due to synced fast-forwarding");
        zinfo->profSimTime->transition(PROF_FF);
        while (zinfo->globalSyncedFFProcs) usleep(20*1000);
        zinfo->profSimTime->transition(PROF_WEAVE);
        info("Synced fast-forwarding done, resuming simulation");
    }

    CheckForTermination();
    zinfo->contentionSim->simulatePhase(zinfo->globPhaseCycles + zinfo->phaseLength);
    zinfo->eventQueue->tick();
    zinfo->profSimTime->transition(PROF_BOUND);
}


uint32_t TakeBarrier(uint32_t tid, uint32_t cid) {
    uint32_t newCid = zinfo->sched->sync(procIdx, tid, cid);
    clearCid(tid); //this is after the sync for a hack needed to make EndOfPhase reliable
    setCid(tid, newCid);

    if (procTreeNode->isInFastForward()) {
        info("Thread %d entering fast-forward", tid);
        clearCid(tid);
        zinfo->sched->leave(procIdx, tid, newCid);
        newCid = INVALID_CID;
        SimThreadFini(tid);
        fPtrs[tid] = GetFFPtrs();
    } else if (zinfo->terminationConditionMet) {
        info("Termination condition met, exiting");
        zinfo->sched->leave(procIdx, tid, newCid);
        SimEnd(); //need to call this on a per-process basis...
    } else if (procTreeNode->isInGroupExit()) {
        // Leave and turn to a nop thread to wait to be killed ...
        clearCid(tid);
        zinfo->sched->leave(procIdx, tid, newCid);
        newCid = INVALID_CID;
        fPtrs[tid] = nopPtrs;
    } else {
        // Set fPtrs to those of the new core after possible context switch
        fPtrs[tid] = cores[tid]->GetFuncPtrs();
    }

    return newCid;
}

/* ===================================================================== */

#if 0
static void PrintIp(THREADID tid, ADDRINT ip) {
    if (zinfo->globPhaseCycles > 1000000000L /*&& zinfo->globPhaseCycles < 1000030000L*/) {
        info("[%d] %ld 0x%lx", tid, zinfo->globPhaseCycles, ip);
    }
}
#endif

void PrepareNextInstruction(THREADID tid, INS ins, ADDRINT instAddr, struct BasicBlockLoadStore *loadStore,
        struct BranchInformation *branchInfo) {
    //Uncomment to print an instruction trace
    //INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)PrintIp, IARG_THREAD_ID, IARG_REG_VALUE, REG_INST_PTR, IARG_END);

    if (!procTreeNode->isInFastForward() || !zinfo->ffReinstrument) {
        void (*LoadStoreFuncPtr)(THREADID, ADDRINT) = nullptr;

        bool isLoad = Decoder::riscvInsIsLoad(ins);
        bool isStore = Decoder::riscvInsIsStore(ins);
        if (isLoad) {
            LoadStoreFuncPtr = IndirectLoadSingle;
        }
        if (isStore) {
            LoadStoreFuncPtr = IndirectStoreSingle;
        }
        if (isLoad || isStore) {
            struct BasicBlockLoadStore *loadStoreList = loadStore;
            while (loadStoreList != nullptr) {
                assert(loadStoreList->entryValid);
                LoadStoreFuncPtr(tid, loadStoreList->addr1);
                loadStoreList = loadStore->next;
            }
        }

        if (Decoder::riscvInsIsBranch(ins)) {
            uint8_t firstTwoBits = ins & 0x03;
            uint8_t nextPcAdd = 2;
            if (firstTwoBits == 0x03) {
                nextPcAdd = 4;
            }
            IndirectRecordBranch(tid, instAddr, branchInfo->branchTaken,
                branchInfo->branchTakenNpc, instAddr + nextPcAdd);
        }
    }

    //Intercept and process magic ops
    /* xchg %rcx, %rcx is our chosen magic op. It is effectively a NOP, but it
     * is never emitted by any x86 compiler, as they use other (recommended) nop
     * instructions or sequences.
     */
    // if (INS_IsXchg(ins) && INS_OperandReg(ins, 0) == REG_RCX && INS_OperandReg(ins, 1) == REG_RCX) {
    //     //info("Instrumenting magic op");
    //     INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR) HandleMagicOp, IARG_THREAD_ID, IARG_REG_VALUE, REG_ECX, IARG_END);
    // }

    // if (INS_Opcode(ins) == XED_ICLASS_CPUID) {
    //    INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR) FakeCPUIDPre, IARG_THREAD_ID, IARG_REG_VALUE, REG_EAX, IARG_REG_VALUE, REG_ECX, IARG_END);
    //    INS_InsertCall(ins, IPOINT_AFTER, (AFUNPTR) FakeCPUIDPost, IARG_THREAD_ID, IARG_REG_REFERENCE, REG_EAX,
    //            IARG_REG_REFERENCE, REG_EBX, IARG_REG_REFERENCE, REG_ECX, IARG_REG_REFERENCE, REG_EDX, IARG_END);
    // }

    // if (INS_IsRDTSC(ins)) {
    //     //No pre; note that this also instruments RDTSCP
    //     INS_InsertCall(ins, IPOINT_AFTER, (AFUNPTR) FakeRDTSCPost, IARG_THREAD_ID, IARG_REG_REFERENCE, REG_EAX, IARG_REG_REFERENCE, REG_EDX, IARG_END);
    // }
}

static std::atomic<bool> activeThreads[MAX_THREADS];  // set in ThreadStart, reset in ThreadFini, we need this for exec() (see FollowChild)
#ifdef HARD_CODED_TRACE_TEST
static std::vector<std::queue<struct FrontendTrace>> queuePerThread;
static std::vector<std::unique_ptr<std::mutex>> queueMutexPerThread;
static std::vector<std::unique_ptr<std::condition_variable>> queueHasDataPerThread;
#endif

void ThreadStart(THREADID tid);

void TraceThreadInit(std::vector<std::thread> &threads, int which) {
#ifdef HARD_CODED_TRACE_TEST
    queuePerThread.emplace_back();

    auto mutexPtr = std::make_unique<std::mutex>();
    queueMutexPerThread.push_back(std::move(mutexPtr));

    auto cvPtr = std::make_unique<std::condition_variable>();
    queueHasDataPerThread.push_back(std::move(cvPtr));
#endif
    threads.emplace_back(ThreadStart, which);
    while (!activeThreads[0].load());
}

#ifdef HARD_CODED_TRACE_TEST
void Trace(THREADID tid, struct FrontendTrace trace) {
    {
        std::unique_lock<std::mutex> lock(*queueMutexPerThread[tid]);
        queuePerThread[tid].push(trace);
    }
    queueHasDataPerThread[tid]->notify_one();
}
#endif

/* ===================================================================== */

void SimThreadStart(THREADID tid) {
    info("Thread %d starting", tid);
    if (tid > MAX_THREADS) panic("tid > MAX_THREADS");
    zinfo->sched->start(procIdx, tid, procTreeNode->getMask());
    activeThreads[tid].store(true);

    //Pinning
#if 0
    if (true) {
        uint32_t nprocs = sysconf(_SC_NPROCESSORS_ONLN);
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(tid % nprocs, &cpuset);
        //HMM, can we do this? I doubt it
        //int result = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
        //Since we're running multiprocess, this suffices for now:
        int result = sched_setaffinity(getpid(), sizeof(cpu_set_t), &cpuset);
        assert(result == 0);
    }
#endif

    //Initialize this thread's process-local data
    fPtrs[tid] = joinPtrs; //delayed, MT-safe barrier join
    clearCid(tid); //just in case, set an invalid cid
}

void ThreadStart(THREADID tid) {
    IPCHandler ipcHandler(tid);

    /* This should only fire for the first thread; I know this is a callback,
     * everything is serialized etc; that's the point, we block everything.
     * It's here and not in main() because that way the auxiliary threads can
     * start.
     */
    if (procTreeNode->isInPause()) {
        futex_lock(&zinfo->pauseLocks[procIdx]);  // initialize
        info("Pausing until notified");
        futex_lock(&zinfo->pauseLocks[procIdx]);  // block
        procTreeNode->exitPause();
        info("Unpaused");
    }

    if (procTreeNode->isInFastForward()) {
        info("FF thread %d starting", tid);
        fPtrs[tid] = GetFFPtrs();
    } else if (zinfo->registerThreads) {
        info("Shadow thread %d starting", tid);
        fPtrs[tid] = nopPtrs;
    } else {
        //Start normal thread
        SimThreadStart(tid);
    }

    ipcHandler.waitAccept();

    while (true) {
#ifdef HARD_CODED_TRACE_TEST
        std::unique_lock<std::mutex> lock(*queueMutexPerThread[tid]);
        queueHasDataPerThread[tid]->wait(lock, [tid] {
            return !queuePerThread[tid].empty() || !activeThreads[tid].load();
        });
#endif
        if (!activeThreads[tid].load()) {
            break;
        }
#ifdef HARD_CODED_TRACE_TEST
        while (!queuePerThread[tid].empty()) {
            auto trace = queuePerThread[tid].front();
            queuePerThread[tid].pop();
#else
        auto tracePtr = ipcHandler.receiveTrace();
        auto &trace = *tracePtr;
#endif
            if (!procTreeNode->isInFastForward() || !zinfo->ffReinstrument) {
                // Visit every basic block in the trace
                for (size_t i = 0; i < trace.count; i++) {
                    struct BasicBlock &bbl = trace.blocks[i];
                    BblInfo* bblInfo = Decoder::decodeBbl(bbl, zinfo->oooDecode);
                    IndirectBasicBlock(tid, bbl.virtualPc, bblInfo);
                }
            }
        
            for (size_t i = 0; i < trace.count; i++) {
                struct BasicBlock &bbl = trace.blocks[i];
                bbl.resetProgramIndex();
                size_t instIndex = 0;
                    PrepareNextInstruction(tid, ins, bbl.virtualPc + instIndex, ldstList, &bbl.branchInfo);
                }
            }
#ifdef HARD_CODED_TRACE_TEST
        }
#endif
    }
}

void SimThreadFini(THREADID tid) {
#ifdef HARD_CODED_TRACE_TEST
    while (!queuePerThread[tid].empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif
    activeThreads[tid].store(false);
#ifdef HARD_CODED_TRACE_TEST
    queueHasDataPerThread[tid]->notify_one();
#endif
    zinfo->sched->finish(procIdx, tid);
    cids[tid] = UNINITIALIZED_CID; //clear this cid, it might get reused
}

void ThreadFini(THREADID tid) {
    //NOTE: Thread has no valid cid here!
    if (fPtrs[tid].type == FPTR_NOP) {
        info("Shadow/NOP thread %d finished", tid);
        return;
    } else {
        SimThreadFini(tid);
        info("Thread %d finished", tid);
    }
}

/* Fork and exec instrumentation */

//For funky macro stuff
#define QUOTED_(x) #x
#define QUOTED(x) QUOTED_(x)

void SimEnd() {
    if (__sync_bool_compare_and_swap(&perProcessEndFlag, 0, 1) == false) { //failed, note DEPENDS ON STRONG CAS
        while (true) { //sleep until thread that won exits for us
            struct timespec tm;
            tm.tv_sec = 1;
            tm.tv_nsec = 0;
            nanosleep(&tm, nullptr);
        }
    }

    //at this point, we're in charge of exiting our whole process, but we still need to race for the stats

    //per-process
#ifdef BBL_PROFILING
    Decoder::dumpBblProfile();
#endif

    //global
    bool lastToFinish = procTreeNode->notifyEnd();
    (void) lastToFinish; //make gcc happy; not needed anymore, since proc 0 dumps stats

    if (procIdx == 0) {
        //Done to preserve the scheduler and contention simulation internal threads
        if (zinfo->globalActiveProcs) {
            info("Delaying termination until all other processes finish");
            while (zinfo->globalActiveProcs) usleep(100*1000);
            info("All other processes done, terminating");
        }

        info("Dumping termination stats");
        zinfo->trigger = 20000;
        for (StatsBackend* backend : *(zinfo->statsBackends)) backend->dump(false /*unbuffered, write out*/);
        for (AccessTraceWriter* t : *(zinfo->traceWriters)) t->dump(false);  // flushes trace writer

        if (zinfo->sched) zinfo->sched->notifyTermination();
    }

    //Uncomment when debugging termination races, which can be rare because they are triggered by threads of a dying process
    //sleep(5);

    exit(0);
}


// Magic ops interface
/* TODO: In the future, we might want to return values to the program.
 * This is definitely doable, but there is no use for it now.
 */
#define ZSIM_MAGIC_OP_ROI_BEGIN         (1025)
#define ZSIM_MAGIC_OP_ROI_END           (1026)
#define ZSIM_MAGIC_OP_REGISTER_THREAD   (1027)
#define ZSIM_MAGIC_OP_HEARTBEAT         (1028)

// VOID HandleMagicOp(THREADID tid, ADDRINT op) {
//     switch (op) {
//         case ZSIM_MAGIC_OP_ROI_BEGIN:
//             if (!zinfo->ignoreHooks) {
//                 //TODO: Test whether this is thread-safe
//                 futex_lock(&zinfo->ffLock);
//                 if (procTreeNode->isInFastForward()) {
//                     info("ROI_BEGIN, exiting fast-forward");
//                     ExitFastForward();
//                 } else {
//                     warn("Ignoring ROI_BEGIN magic op, not in fast-forward");
//                 }
//                 futex_unlock(&zinfo->ffLock);
//             }
//             return;
//         case ZSIM_MAGIC_OP_ROI_END:
//             if (!zinfo->ignoreHooks) {
//                 //TODO: Test whether this is thread-safe
//                 futex_lock(&zinfo->ffLock);
//                 if (procTreeNode->getSyncedFastForward()) {
//                     warn("Ignoring ROI_END magic op on synced FF to avoid deadlock");
//                 } else if (!procTreeNode->isInFastForward()) {
//                     info("ROI_END, entering fast-forward");
//                     EnterFastForward();
//                     //If we don't do this, we'll enter FF on the next phase. Which would be OK, except with synced FF
//                     //we stay in the barrier forever. And deadlock. And the deadlock code does nothing, since we're in FF
//                     //So, force immediate entry if we're sync-ffwding
//                     if (procTreeNode->getSyncedFastForward()) {
//                         info("Thread %d entering fast-forward (immediate)", tid);
//                         uint32_t cid = getCid(tid);
//                         assert(cid != INVALID_CID);
//                         clearCid(tid);
//                         zinfo->sched->leave(procIdx, tid, cid);
//                         SimThreadFini(tid);
//                         fPtrs[tid] = GetFFPtrs();
//                     }
//                 } else {
//                     warn("Ignoring ROI_END magic op, already in fast-forward");
//                 }
//                 futex_unlock(&zinfo->ffLock);
//             }
//             return;
//         case ZSIM_MAGIC_OP_REGISTER_THREAD:
//             if (!zinfo->registerThreads) {
//                 info("Thread %d: Treating REGISTER_THREAD magic op as NOP", tid);
//             } else {
//                 if (fPtrs[tid].type == FPTR_NOP) {
//                     SimThreadStart(tid);
//                 } else {
//                     warn("Thread %d: Treating REGISTER_THREAD magic op as NOP, thread already registered", tid);
//                 }
//             }
//             return;
//         case ZSIM_MAGIC_OP_HEARTBEAT:
//             procTreeNode->heartbeat(); //heartbeats are per process for now
//             return;

//         // HACK: Ubik magic ops
//         case 1029:
//         case 1030:
//         case 1031:
//         case 1032:
//         case 1033:
//             return;
//         default:
//             panic("Thread %d issued unknown magic op %ld!", tid, op);
//     }
// }

// //CPUIID faking
// static uint32_t cpuidEax[MAX_THREADS];
// static uint32_t cpuidEcx[MAX_THREADS];

// VOID FakeCPUIDPre(THREADID tid, REG eax, REG ecx) {
//     //info("%d precpuid", tid);
//     cpuidEax[tid] = eax;
//     cpuidEcx[tid] = ecx;
// }

// VOID FakeCPUIDPost(THREADID tid, ADDRINT* eax, ADDRINT* ebx, ADDRINT* ecx, ADDRINT* edx) {
//     uint32_t eaxIn = cpuidEax[tid];
//     uint32_t ecxIn = cpuidEcx[tid];

//     // Point to record at same (eax,ecx) or immediately before
//     CpuIdRecord val = {eaxIn, ecxIn, (uint32_t)-1, (uint32_t)-1, (uint32_t)-1, (uint32_t)-1};
//     CpuIdRecord* pos = std::lower_bound(cpuid_core2, cpuid_core2+(sizeof(cpuid_core2)/sizeof(CpuIdRecord)), val);
//     if (pos->eaxIn > eaxIn) {
//         assert(pos > cpuid_core2);
//         pos--;
//     }
//     assert(pos->eaxIn <= eaxIn);
//     assert(pos->ecxIn <= ecxIn);

//     //info("%x %x : %x %x / %x %x %x %x", eaxIn, ecxIn, pos->eaxIn, pos->ecxIn, pos->eax, pos->ebx, pos->ecx, pos->edx);

//     uint32_t eaxOut = pos->eax;
//     uint32_t ebxOut = pos->ebx;

//     // patch eax to give the number of cores
//     if (eaxIn == 4) {
//         uint32_t ncpus = cpuenumNumCpus(procIdx);
//         uint32_t eax3126 = ncpus - 1;
//         // Overflowing 6 bits?
//         if (zinfo->numCores > 64) eax3126 = 63; //looked into swarm2.csail (4P Westmere-EX, 80 HTs), it sets this to 63
//         eaxOut = (eaxOut & ((1<<26)-1)) | (eax3126<<26);
//     }

//     // HT siblings and APIC (core) ID (apparently used; seems Intel-specific)
//     if (eaxIn == 0x1) {
//         uint32_t cid = getCid(tid);
//         uint32_t cpu = cpuenumCpu(procIdx, cid);
//         uint32_t ncpus = cpuenumNumCpus(procIdx);
//         uint32_t siblings = MIN(ncpus, (uint32_t)255);
//         uint32_t apicId = (cpu < ncpus)? MIN(cpu, (uint32_t)255) : 0 /*not scheduled, ffwd?*/;
//         ebxOut = (ebxOut & 0xffff) | (siblings << 16) | (apicId << 24);
//     }

//     //info("[%d] postcpuid, inEax 0x%x, pre 0x%lx 0x%lx 0x%lx 0x%lx", tid, eaxIn, *eax, *ebx, *ecx, *edx);
//     //Preserve high bits
//     *reinterpret_cast<uint32_t*>(eax) = eaxOut;
//     *reinterpret_cast<uint32_t*>(ebx) = ebxOut;
//     *reinterpret_cast<uint32_t*>(ecx) = pos->ecx;
//     *reinterpret_cast<uint32_t*>(edx) = pos->edx;
//     //info("[%d] postcpuid, inEax 0x%x, post 0x%lx 0x%lx 0x%lx 0x%lx", tid, eaxIn, *eax, *ebx, *ecx, *edx);
// }


// //RDTSC faking
// VOID FakeRDTSCPost(THREADID tid, REG* eax, REG* edx) {
//     if (fPtrs[tid].type == FPTR_NOP) return; //avoid virtualizing NOP threads.

//     uint32_t cid = getCid(tid);
//     uint64_t curCycle = VirtGetPhaseRDTSC();
//     if (cid < zinfo->numCores) {
//         curCycle += zinfo->cores[cid]->getPhaseCycles();
//     }

//     uint32_t lo = (uint32_t)curCycle;
//     uint32_t hi = (uint32_t)(curCycle >> 32);

//     assert((((uint64_t)hi) << 32) + lo == curCycle);

//     //uint64_t origTSC = (((uint64_t)*edx) << 32) + (uint32_t)*eax;
//     //info("[t%d/c%d] Virtualizing RDTSC, pre = %x %x (%ld), post = %x %x (%ld)", tid, cid, *edx, *eax, origTSC, hi, lo, curCycle);

//     *eax = (REG)lo;
//     *edx = (REG)hi;
// }

/* Fast-forward control */

// Helper class, enabled the FFControl thread to sync with the phase end code
class SyncEvent: public Event {
    private:
        lock_t arrivalLock;
        lock_t leaveLock;

    public:
        SyncEvent() : Event(0 /*one-shot*/) {
            futex_init(&arrivalLock);
            futex_init(&leaveLock);

            futex_lock(&arrivalLock);
            futex_lock(&leaveLock);
        }

        // Blocks until callback()
        void wait() {
            futex_lock(&arrivalLock);
        }

        // Unblocks thread that called wait(), blocks until signal() called
        // Resilient against callback-wait races (wait does not block if it's
        // called afteer callback)
        void callback() {
            futex_unlock(&arrivalLock);
            futex_lock(&leaveLock);
        }

        // Unblocks thread waiting in callback()
        void signal() {
            futex_unlock(&leaveLock);
        }
};

void FFThread(void* arg) {
    futex_lock(&zinfo->ffToggleLocks[procIdx]); //initialize
    info("FF control Thread TID %ld", syscall(SYS_gettid));

    while (true) {
        //block ourselves until someone wakes us up with an unlock
        bool locked = futex_trylock_nospin_timeout(&zinfo->ffToggleLocks[procIdx], 5*BILLION /*5s timeout*/);

        if (!locked) { //timeout
            if (zinfo->terminationConditionMet) {
                info("Terminating FF control thread");
                SimEnd();
                panic("Should not be reached");
            }
            //info("FF control thread wakeup");
            continue;
        }

        futex_lock(&zinfo->ffLock);
        if (procTreeNode->isInFastForward()) {
            info("Exiting fast forward");
            ExitFastForward();
        } else {
            SyncEvent* syncEv = new SyncEvent();
            zinfo->eventQueue->insert(syncEv); //will run on next phase
            info("Pending fast-forward entry, waiting for end of phase (%ld phases)", zinfo->numPhases);

            futex_unlock(&zinfo->ffLock);
            syncEv->wait();
            //At this point the thread thet triggered the end of phase is blocked inside of EndOfPhaseActions
            futex_lock(&zinfo->ffLock);
            if (!procTreeNode->isInFastForward()) {
                info("End of phase %ld, entering FF", zinfo->numPhases);
                EnterFastForward();
            } else {
                info("FF control thread called on end of phase, but someone else (program?) already entered ffwd");
            }
            syncEv->signal(); //unblock thread in EndOfPhaseActions
        }
        futex_unlock(&zinfo->ffLock);
    }
    panic("Should not be reached!");
}

/* linux kernel memset */
static uint32_t bbl0_code[] = {
    0x00c286b3, 0x00b28023, 0xede30285, 0x0000fed2
};

static struct BasicBlock bbl0, bbl1;
static struct FrontendTrace testTrace0, testTrace1;

void buildTestTrace() {
    bbl0.codeBytes = 14;
    bbl0.code = (uint8_t *) bbl0_code;
    bbl0.loadStore = (struct BasicBlockLoadStore *)malloc(4 * sizeof(struct BasicBlockLoadStore));
    bbl0.branchInfo.branchTaken = true;
    bbl0.branchInfo.branchTakenNpc = 0xffffffff8090f174;
    bbl0.virtualPc = 0xffffffff8090f170;
    bbl0.resetProgramIndex();
    bbl0.loadStore[0].entryValid = false;
    bbl0.loadStore[1].entryValid = true;
    bbl0.loadStore[1].addr1 = 0xffffffff8190f170;
    bbl0.loadStore[1].next = nullptr;
    bbl0.loadStore[2].entryValid = false;
    bbl0.loadStore[3].entryValid = false;
    testTrace0.blocks = &bbl0;
    testTrace0.count = 1;

    bbl1.codeBytes = 10;
    bbl1.code = (uint8_t *) (bbl0_code) + 4;
    bbl1.loadStore = (struct BasicBlockLoadStore *)malloc(3 * sizeof(struct BasicBlockLoadStore));
    bbl1.branchInfo.branchTaken = true;
    bbl1.branchInfo.branchTakenNpc = 0xffffffff8090f174;
    bbl1.virtualPc = 0xffffffff8090f174;
    bbl1.resetProgramIndex();
    bbl1.loadStore[0].entryValid = true;
    bbl1.loadStore[0].addr1 = 0x8190fa70; /* force a cache miss */
    bbl1.loadStore[0].next = nullptr;
    bbl1.loadStore[1].entryValid = false;
    bbl1.loadStore[2].entryValid = false;
    testTrace1.blocks = &bbl1;
    testTrace1.count = 1;
}

/* ===================================================================== */

int main(int argc, char *argv[]) {
    procIdx = 0;
    char header[64];
    snprintf(header, sizeof(header), "[S %d] ", procIdx);

    /* argv 1 output directory argv 2 log_to_file argv 3 config file */
    if (argc != 4) {
        std::cout << "You must specify output directory, log to file and config file option" << std::endl;
        std::cout << "Default value: ./ false zsim.cfg" << std::endl;
        return 1;
    }
    char *outputDir = argv[1], *logToFile = argv[2], *configFile = argv[3];

    std::stringstream logfile_ss;
    logfile_ss << outputDir << "/zsim.log." << procIdx;
    InitLog(header, !strcmp(logToFile, "log_to_file") ? logfile_ss.str().c_str() : nullptr);

    //If parent dies, kill us
    //This avoids leaving strays running in any circumstances, but may be too heavy-handed with arbitrary process hierarchies.
    //If you ever need this disabled, sim.pinOptions = "-injection child" does the trick
    if (syscall(SYS_prctl, PR_SET_PDEATHSIG, 9 /*SIGKILL*/) != 0) {
        panic("prctl() failed");
    }

    info("Started instance");

    Config conf(configFile);
    uint32_t gmSize = conf.get<uint32_t>("sim.gmMBytes", (1<<10) /*default 1024MB*/);
    info("Creating global segment, %d MBs", gmSize);
    int shmid = gm_init(((size_t)gmSize) << 20 /*MB to Bytes*/);
    info("Global segment shmid = %d", shmid);

    bool masterProcess = false;
    if (procIdx == 0 && !gm_isready()) {  // process 0 can exec() without fork()ing first, so we must check gm_isready() to ensure we don't initialize twice
        masterProcess = true;
        SimInit(configFile, outputDir, 0);
    } else {
        while (!gm_isready()) usleep(1000);  // wait till proc idx 0 initializes everything
        zinfo = static_cast<GlobSimInfo*>(gm_get_glob_ptr());
    }

    //Attach to debugger if needed (master process does so in SimInit, to be able to debug initialization)
    //NOTE: Pin fails to follow exec()'s when gdb is attached. The simplest way to avoid it is to kill the debugger manually before an exec(). If this is common, we could automate it
    if (!masterProcess && zinfo->attachDebugger) {
        notifyHarnessForDebugger(zinfo->harnessPid, zinfo->debugPortId);
    }

    assert((uint32_t)procIdx < zinfo->numProcs);
    procTreeNode = zinfo->procArray[procIdx];
    if (!masterProcess) procTreeNode->notifyStart(); //masterProcess notifyStart is called in init() to avoid races
    assert(procTreeNode->getProcIdx() == (uint32_t)procIdx); //must be consistent

    trace(Process, "SHM'd global segment, starting");

    assert(zinfo->phaseLength > 0);
    assert(zinfo->maxPhases >= 0);
    assert(zinfo->statsPhaseInterval >= 0);

    perProcessEndFlag = 0;

    lineBits = ilog2(zinfo->lineSize);
    pageBits = ilog2(zinfo->pageSize);
    procMask = ((uint64_t)procIdx) << (64-lineBits);

    //Initialize process-local per-thread state, even if ThreadStart does so later
    for (uint32_t i = 0; i < MAX_THREADS; i++) {
        fPtrs[i] = joinPtrs;
        cids[i] = UNINITIALIZED_CID;
    }

    info("Started process, PID %d", getpid()); //NOTE: external scripts expect this line, please do not change without checking first

    info("procMask: 0x%lx", procMask);

    if (zinfo->sched) zinfo->sched->processCleanup(procIdx);

    FFIInit();

    std::vector<std::thread> threads;
    threads.emplace_back(FFThread, nullptr);

    TraceThreadInit(threads, 0);

#ifdef HARD_CODED_TRACE_TEST
    buildTestTrace();
    Trace(0, testTrace0);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    Trace(0, testTrace1);
    EndOfPhaseActions();
#else
    for (auto &thd: threads) {
        thd.join();
    }
#endif
    ThreadFini(0);
    SimEnd();
    return 0;
}

