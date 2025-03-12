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

#ifndef DECODER_H_
#define DECODER_H_

#include <cstdint>
#include <cstddef>
#include <vector>

// Uncomment to get a count of BBLs run. This is currently used to get a distribution of inaccurate instructions decoded that are actually run
// NOTE: This is not multiprocess-safe
// #define BBL_PROFILING
// #define PROFILE_ALL_INSTRS

// uop reg limits
#define MAX_UOP_SRC_REGS 2
#define MAX_UOP_DST_REGS 2

/* NOTE this uses stronly typed enums, a C++11 feature. This saves a bunch of typecasts while keeping UopType enums 1-byte long.
 * If you use gcc < 4.6 or some other compiler, either go back to casting or lose compactness in the layout.
 */
enum UopType : uint8_t {UOP_GENERAL, UOP_LOAD, UOP_STORE, UOP_STORE_ADDR, UOP_FENCE};

struct DynUop {
    uint16_t rs[MAX_UOP_SRC_REGS];
    uint16_t rd[MAX_UOP_DST_REGS];
    uint16_t lat;
    uint16_t decCycle;
    UopType type; //1 byte
    uint8_t portMask;
    uint8_t extraSlots; //FU exec slots
    uint8_t pad; //pad to 4-byte multiple

    void clear();
};  // 16 bytes. TODO(dsm): check performance with wider operands

struct DynBbl {
#ifdef BBL_PROFILING
    uint64_t bblIdx;
#endif
    uint64_t addr;
    uint32_t uops;
    uint32_t approxInstrs;
    DynUop uop[1];

    static uint32_t bytes(uint32_t uops) {
        return offsetof(DynBbl, uop) + sizeof(DynUop)*uops /*wtf... offsetof doesn't work with uop[uops]*/;
    }

    void init(uint64_t _addr, uint32_t _uops, uint32_t _approxInstrs) {
        // NOTE: this is a POD type, so we don't need to call a constructor; otherwise, we should use placement new
        uops = _uops;
        approxInstrs = _approxInstrs;
    }
};

struct BblInfo;  // defined in core.h

/* These are absolute maximums per instruction. If there is some non-conforming instruction, either increase these limits or
 * treat it as a special case.
 */
#define MAX_INSTR_LOADS 4
#define MAX_INSTR_REG_READS 4
#define MAX_INSTR_REG_WRITES 4
#define MAX_INSTR_STORES 4

#define MAX_UOPS_PER_INSTR 12  // technically, even full decoders produce 1-4 uops; we increase this for common microsequenced instructions (e.g. xchg).

/* Temporary register offsets */
#define REG_LOAD_TEMP (REG_LAST + 1)  // REG_LAST defined by PIN
#define REG_STORE_TEMP (REG_LOAD_TEMP + MAX_INSTR_LOADS)
#define REG_STORE_ADDR_TEMP (REG_STORE_TEMP + MAX_INSTR_STORES)
#define REG_EXEC_TEMP (REG_STORE_ADDR_TEMP + MAX_INSTR_STORES)

#define MAX_REGISTERS (REG_EXEC_TEMP + 64)

typedef std::vector<DynUop> DynUopVec;
typedef uint32_t INS;

/* RISC-V definitions */
#define RISCV_OPCODE_ATOMIC                 0x2f
#define RISCV_OPCODE_INTEGER                0x33
#define RISCV_OPCODE_INTEGER_IMM            0x13
#define RISCV_OPCODE_INTEGER_32             0x3b
#define RISCV_OPCODE_INTEGER_IMM_32         0x1b
#define RISCV_OPCODE_LOAD                   0x03
#define RISCV_OPCODE_STORE                  0x23
#define RISCV_OPCODE_BRANCH                 0x63
#define RISCV_OPCODE_JAL                    0x6f
#define RISCV_OPCODE_JALR                   0x67
#define RISCV_OPCODE_LUI                    0x37
#define RISCV_OPCODE_AUIPC                  0x67
#define RISCV_OPCODE_SYSTEM                 0x73
#define RISCV_OPCODE_FENCE                  0x0f
#define RISCV_OPCODE_LOAD_FP                0x07
#define RISCV_OPCODE_STORE_FP               0x27
#define RISCV_OPCODE_MADD_FP                0x43
#define RISCV_OPCODE_MSUB_FP                0x47
#define RISCV_OPCODE_NMSUB_FP               0x4b
#define RISCV_OPCODE_NMADD_FP               0x4f
#define RISCV_OPCODE_FP                     0x53
#define RISCV_OPCODE_C0                     0x0
#define RISCV_OPCODE_C1                     0x1
#define RISCV_OPCODE_C2                     0x2
#define RISCV_OPCODE_VECTOR_LOAD            0x07
#define RISCV_OPCODE_VECTOR_STORE           0x27
#define RISCV_OPCODE_VECTOR_ARITH           0x57

//Nehalem-style decoder. Fully static for now
class Decoder {
    private:
        struct Instr {
            INS ins;

            uint32_t loadOps[MAX_INSTR_LOADS];
            uint32_t numLoads;

            //These contain the register indices; by convention, flags registers are stored last
            uint32_t inRegs[MAX_INSTR_REG_READS];
            uint32_t numInRegs;
            uint32_t outRegs[MAX_INSTR_REG_WRITES];
            uint32_t numOutRegs;

            uint32_t storeOps[MAX_INSTR_STORES];
            uint32_t numStores;

            explicit Instr(INS _ins);

            private:
                //Put registers in some canonical order -- non-flags first
                // void reorderRegs(uint32_t* regArray, uint32_t numRegs);
        };

    public:
        //If oooDecoding is true, produces a DynBbl with DynUops that can be used in OOO cores
        static BblInfo* decodeBbl(BBL bbl, bool oooDecoding);

#ifdef BBL_PROFILING
        static void profileBbl(uint64_t bblIdx);
        static void dumpBblProfile();
#endif

    private:
        static uint8_t riscvInsOpCode(INS ins);
        static uint8_t riscvInsFunct3(INS ins);
        static uint8_t riscvInsFunct7(INS ins);
        static uint8_t riscvInsIsAtomic(INS ins);
        static uint8_t riscvInsArithRd(INS ins);
        static uint8_t riscvInsArithRs1(INS ins);
        static uint8_t riscvInsArithRs2(INS ins);
        static uint8_t riscvCompressedRegDecode(uint8_t reg);
        //Return true if inaccurate decoding, false if accurate
        static bool decodeInstr(INS ins, DynUopVec& uops);

        /* Every emit function can produce 0 or more uops; it returns the number of uops. These are basic templates to make our life easier */

        //By default, these emit to temporary registers that depend on the index; this can be overriden, e.g. for moves
        static void emitLoad(DynUopVec& uops, uint16_t destReg, uint16_t baseReg);
        static void emitStore(DynUopVec& uops, uint16_t dataReg, uint16_t addrReg);

        //Emits a load-store fence uop
        static void emitFence(DynUopVec& uops, uint32_t lat);

        static void emitExecUop(uint32_t rs0, uint32_t rs1, uint32_t rd0, uint32_t rd1,
                DynUopVec& uops, uint32_t lat, uint8_t ports, uint8_t extraSlots = 0);

        /* Instruction emits */

        static void emitBasicMove(DynUopVec& uops, uint16_t rd, uint32_t lat, uint8_t ports);

        /* Specific cases */
        static void emitXchg(DynUopVec& uops, uint16_t rd, uint16_t rs1, uint16_t rs2);
        static void emitMul(DynUopVec& uops, uint16_t rd, uint16_t rs1, uint16_t rs2);
        static void emitDiv(DynUopVec& uops, uint8_t width, uint16_t rd, uint16_t rs1, uint16_t rs2);

        /* Other helper functions */
        static void reportUnhandledCase(Instr& instr, const char* desc);

        /* Macro-op (ins) fusion */
        static bool canFuse(INS ins);
        static bool decodeFusedInstrs(INS ins, DynUopVec& uops);
};

typedef uint64_t THREADID;
typedef uint64_t ADDRINT;
typedef bool BOOL;

#endif  // DECODER_H_
