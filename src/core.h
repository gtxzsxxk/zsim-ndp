/** $lic$
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
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

#ifndef CORE_H_
#define CORE_H_

#include <stdint.h>
#include "g_std/g_string.h"
#include "stats.h"
#include <memory>

typedef uint32_t INS;
typedef uint64_t THREADID;
typedef uint64_t ADDRINT;
typedef bool BOOL;

struct BranchInformation {
    uint8_t branchTaken;
    ADDRINT branchTakenNpc;
};

struct BasicBlockLoadStore {
    ADDRINT addr1;
    ADDRINT addr2;
    ADDRINT addr3;
    uint64_t value;
    uint8_t entryValid;
    struct BasicBlockLoadStore *next;
};

struct BasicBlock {
    size_t codeBytes;
    uint8_t *code;
    size_t loadStores;
    struct BasicBlockLoadStore *loadStore;
    struct BranchInformation branchInfo;
    uint64_t virtualPc;
    uint64_t midgardPc;
    uint64_t physicalPc;
    size_t programIndex;

    void resetProgramIndex() {
        programIndex = 0;
    }
    
    INS getHeadInstruction(size_t *index = nullptr, uint8_t *instLength = nullptr, bool lookahead = false) {
        if (programIndex >= codeBytes) {
            programIndex += 4;
            return 0xffffffff;
        }
        if (index) {
            *index = programIndex;
        }
        INS tryFetch = *(INS *)(code + programIndex);
        uint8_t firstTwoBits = tryFetch & 0x03;
        switch (firstTwoBits) {
            case 0x0:
            case 0x1:
            case 0x2:
                if (instLength) {
                    *instLength = 2;
                }
                programIndex += lookahead ? 0 : 2;
                return tryFetch & 0xffff;
            default:
                if (instLength) {
                    *instLength = 4;
                }
                programIndex += lookahead ? 0 : 4;
                return tryFetch;
        }
    }

    size_t getInstructionCount() {
        size_t ret = 0;
        resetProgramIndex();
        for (getHeadInstruction(); !endOfBlock(); getHeadInstruction()) {
            ret++;
        }
        resetProgramIndex();
        return ret;
    }

    bool endOfBlock() {
        return programIndex > codeBytes;
    }

    ~BasicBlock() {
        delete[] code;
        for (size_t i = 0; i < loadStores; i++) {
            auto next = loadStore[i].next;
            while (next) {
                auto cur = next;
                next = next->next;
                delete[] cur;
            }
        }
        delete[] loadStore;
    }
};

struct FrontendTrace {
    struct BasicBlock *blocks;
    size_t count;

    FrontendTrace() = default;

    ~FrontendTrace() {
        delete[] blocks;
    }
};

struct BblInfo;

/* Analysis function pointer struct
 * As an artifact of having a shared code cache, we need these to be the same for different core types.
 */
struct InstrFuncPtrs {  // NOLINT(whitespace)
    void (*loadPtr)(THREADID, ADDRINT);
    void (*storePtr)(THREADID, ADDRINT);
    void (*bblPtr)(THREADID, ADDRINT, BblInfo*);
    void (*branchPtr)(THREADID, ADDRINT, BOOL, ADDRINT, ADDRINT);
    // Same as load/store functions, but last arg indicated whether op is executing
    void (*predLoadPtr)(THREADID, ADDRINT, BOOL);
    void (*predStorePtr)(THREADID, ADDRINT, BOOL);
    uint64_t type;
    uint64_t pad[1];
    //NOTE: By having the struct be a power of 2 bytes, indirect calls are simpler (w/ gcc 4.4 -O3, 6->5 instructions, and those instructions are simpler)
};


//TODO: Switch type to an enum by using sizeof macros...
#define FPTR_ANALYSIS (0L)
#define FPTR_JOIN (1L)
#define FPTR_NOP (2L)
#define FPTR_RETRY (3L)

//Generic core class

class Core : public GlobAlloc {
    private:
        uint64_t lastUpdateCycles;
        uint64_t lastUpdateInstrs;

    protected:
        g_string name;

    public:
        explicit Core(g_string& _name) : lastUpdateCycles(0), lastUpdateInstrs(0), name(_name) {}

        virtual uint64_t getInstrs() const = 0; // typically used to find out termination conditions or dumps
        virtual uint64_t getPhaseCycles() const = 0; // used by RDTSC faking --- we need to know how far along we are in the phase, but not the total number of phases
        virtual uint64_t getCycles() const = 0;

        virtual void initStats(AggregateStat* parentStat) = 0;
        virtual void contextSwitch(int32_t gid) = 0; //gid == -1 means descheduled, otherwise this is the new gid

        //Called by scheduler on every leave and join action, before barrier methods are called
        virtual void leave() {}
        virtual void join() {}

        //Contention simulation interface
        virtual bool needsCSim() const { return false; }
        virtual void cSimStart() {}
        virtual void cSimEnd() {}

        virtual InstrFuncPtrs GetFuncPtrs() = 0;
};

#endif  // CORE_H_

