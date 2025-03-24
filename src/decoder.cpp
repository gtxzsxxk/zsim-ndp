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

#include "decoder.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string.h>
#include <string>
#include <vector>
#include "core.h"
#include "locks.h"
#include "log.h"

//PORT defines. You might want to change these to affect scheduling
#define PORT_0 (0x1)
#define PORT_1 (0x2)
#define PORT_2 (0x4)
#define PORT_3 (0x8)
#define PORT_4 (0x10)
#define PORT_5 (0x20)

#define PORTS_015 (PORT_0 | PORT_1 | PORT_5)

void DynUop::clear() {
    memset(this, 0, sizeof(DynUop));  // NOTE: This may break if DynUop becomes non-POD
}

Decoder::Instr::Instr(INS _ins) : ins(_ins), numLoads(0), numInRegs(0), numOutRegs(0), numStores(0) {
    // auto op = riscvOpCode(_ins);
    // bool read = INS_OperandRead(ins, op);
    // bool write = INS_OperandWritten(ins, op);
    // assert(read || write);
    // if (INS_OperandIsMemory(ins, op)) {
    //     if (read) loadOps[numLoads++] = op;
    //     if (write) storeOps[numStores++] = op;
    // } else if (INS_OperandIsReg(ins, op) && INS_OperandReg(ins, op)) { //it's apparently possible to get INS_OperandIsReg to be true and an invalid reg ... WTF Pin?
    //     REG reg = INS_OperandReg(ins, op);
    //     assert(reg);  // can't be invalid
    //     reg = REG_FullRegName(reg);  // eax -> rax, etc; o/w we'd miss a bunch of deps!
    //     if (read) inRegs[numInRegs++] = reg;
    //     if (write) outRegs[numOutRegs++] = reg;
    // }

    // //By convention, we move flags regs to the end
    // reorderRegs(inRegs, numInRegs);
    // reorderRegs(outRegs, numOutRegs);
}

// static inline bool isFlagsReg(uint32_t reg) {
//     return (reg == REG_EFLAGS || reg == REG_FLAGS || reg == REG_MXCSR);
// }

// void Decoder::Instr::reorderRegs(uint32_t* array, uint32_t regs) {
//     if (regs == 0) return;
//     //Unoptimized bubblesort -- when arrays are this short, regularity wins over O(n^2).
//     uint32_t swaps;
//     do {
//         swaps = 0;
//         for (uint32_t i = 0; i < regs-1; i++) {
//             if (isFlagsReg(array[i]) && !isFlagsReg(array[i+1])) {
//                 std::swap(array[i], array[i+1]);
//                 swaps++;
//             }
//         }
//     } while (swaps > 0);
// }

void Decoder::reportUnhandledCase(Instr& instr, const char* desc) {
    warn("Unhandled case: %s | %08x", desc, instr.ins);
}

void Decoder::emitLoad(DynUopVec& uops, uint16_t destReg, uint16_t baseReg) {
    DynUop uop;
    uop.clear();
    uop.rs[0] = baseReg;
    uop.rd[0] = destReg;
    uop.type = UOP_LOAD;
    uop.portMask = PORT_2;
    uops.push_back(uop); //FIXME: The interface should support in-place grow...
}

void Decoder::emitStore(DynUopVec& uops, uint16_t dataReg, uint16_t addrReg) {
    DynUop addrUop;
    addrUop.clear();
    addrUop.rs[0] = dataReg;
    addrUop.rd[0] = addrReg;
    addrUop.lat = 1;
    addrUop.portMask = PORT_3;
    addrUop.type = UOP_STORE_ADDR;
    uops.push_back(addrUop);

    //Emit store uop
    DynUop uop;
    uop.clear();
    uop.rs[0] = addrReg;
    uop.rs[1] = dataReg;
    uop.portMask = PORT_4;
    uop.type = UOP_STORE;
    uops.push_back(uop);
}

void Decoder::emitFence(DynUopVec& uops, uint32_t lat) {
    DynUop uop;
    uop.clear();
    uop.lat = lat;
    uop.portMask = PORT_4; //to the store queue
    uop.type = UOP_FENCE;
    uops.push_back(uop);
}

void Decoder::emitExecUop(uint32_t rs0, uint32_t rs1, uint32_t rd0, uint32_t rd1, DynUopVec& uops, uint32_t lat, uint8_t ports, uint8_t extraSlots) {
    DynUop uop;
    uop.clear();
    uop.rs[0] = rs0;
    uop.rs[1] = rs1;
    uop.rd[0] = rd0;
    uop.rd[1] = rd1;
    uop.lat = lat;
    uop.type = UOP_GENERAL;
    uop.portMask = ports;
    uop.extraSlots = extraSlots;
    uops.push_back(uop);
}

void Decoder::emitBasicMove(DynUopVec& uops, uint16_t rd, uint32_t lat, uint8_t ports) {
    //Note that we can have 0 loads and 0 input registers. In this case, we are loading from an immediate, and we set the input register to 0 so there is no dependence
    emitExecUop(0, 0, rd, 0, uops, lat, ports);
}

void Decoder::emitXchg(DynUopVec& uops, uint16_t rd, uint16_t rs1, uint16_t rs2) {
    emitLoad(uops, rd, rs1);
    emitExecUop(rd, 0, 0, 0, uops, 1, PORTS_015); //r -> temp
    emitExecUop(0, 0, rs2, 0, uops, 1, PORTS_015); // load -> r
    emitStore(uops, 0, rs1); //temp -> out
}

void Decoder::emitMul(DynUopVec& uops, uint16_t rd, uint16_t rs1, uint16_t rs2) {
    emitExecUop(rs1, rs2, rd, 0, uops, 3, PORT_1);
}

void Decoder::emitDiv(DynUopVec& uops, uint8_t width, uint16_t rd, uint16_t rs1, uint16_t rs2) {
    uint32_t lat = 0;
    switch (width) {
        case 8:
            lat = 15;
            break;
        case 16:
            lat = 19;
            break;
        case 32:
            lat = 23;
            break;
        case 64:
            lat = 63;
            break;
        default:
            panic("emitDiv: Invalid reg size");
    }
    uint8_t extraSlots = lat-1;
    emitExecUop(rs1, rs2, rd, 0, uops, lat, PORTS_015, extraSlots);
}

uint8_t Decoder::riscvInsOpCode(INS ins) {
    return ins & 0x7f;
}

uint8_t Decoder::riscvInsFunct3(INS ins) {
    return (ins >> 12) & 0x07;
}

uint8_t Decoder::riscvInsFunct7(INS ins) {
    return (ins >> 25) & 0x7f;
}

uint8_t Decoder::riscvInsIsAtomic(INS ins) {
    return riscvInsOpCode(ins) == RISCV_OPCODE_ATOMIC;
}

uint8_t Decoder::riscvInsArithRd(INS ins) {
    return (ins >> 7) & 0x1f;
}

uint8_t Decoder::riscvInsArithRs1(INS ins) {
    return (ins >> 15) & 0x1f;
}

uint8_t Decoder::riscvInsArithRs2(INS ins) {
    return (ins >> 20) & 0x1f;
}

bool Decoder::riscvInsIsLoad(INS ins) {
    auto opcode = riscvInsOpCode(ins);
    return opcode == RISCV_OPCODE_LOAD || opcode == RISCV_OPCODE_VECTOR_LOAD;
}

bool Decoder::riscvInsIsStore(INS ins) {
    auto opcode = riscvInsOpCode(ins);
    return opcode == RISCV_OPCODE_STORE || opcode == RISCV_OPCODE_VECTOR_STORE;
}

bool Decoder::riscvInsIsBranch(INS ins) {
    auto opcode = riscvInsOpCode(ins);
    return opcode == RISCV_OPCODE_BRANCH;
}

uint8_t Decoder::riscvCompressedRegDecode(uint8_t reg) {
    assert(reg <= 7);
    return reg + 8;
}

bool Decoder::riscvInsIsMemAccess(INS ins) {
    uint8_t opcode = riscvInsOpCode(ins);
    uint8_t funct3 = riscvInsFunct3(ins);
    uint8_t funct7 = riscvInsFunct7(ins);
    switch (opcode) {
        case RISCV_OPCODE_INTEGER:
            break;
        case RISCV_OPCODE_INTEGER_32:
            break;
        case RISCV_OPCODE_INTEGER_IMM:
            break;
        // RV64I I-type word instructions (opcode = 0x1B)
        case RISCV_OPCODE_INTEGER_IMM_32:
            break;
        case RISCV_OPCODE_LOAD:
            return true;
        case RISCV_OPCODE_STORE:
        return true;
        case RISCV_OPCODE_BRANCH:
        case RISCV_OPCODE_JAL:
            break;
        case RISCV_OPCODE_JALR:
            break;
        case RISCV_OPCODE_LUI:
            break;
        case RISCV_OPCODE_AUIPC:
            break;
        case RISCV_OPCODE_ATOMIC:
            return true;
        case RISCV_OPCODE_SYSTEM:
            break;
        case RISCV_OPCODE_FENCE:
            break;
        case RISCV_OPCODE_MADD_FP: // FMADD.S, FMADD.D
        case RISCV_OPCODE_MSUB_FP: // FMSUB.S, FMSUB.D
        case RISCV_OPCODE_NMSUB_FP: // FNMSUB.S, FNMSUB.D
        case RISCV_OPCODE_NMADD_FP: // FNMADD.S, FNMADD.D
            break;
        case RISCV_OPCODE_FP: // Floating point operations
            break;
        /* To some floating compress instructions:
         * We will simply simulated it as a general purposed register
         * since the value does not matter
         */
        case RISCV_OPCODE_C0:
            {
                uint8_t func = (ins >> 13) & 0x07;
                switch(func) {
                    case 0:
                        break;
                    case 1:
                    case 2:
                    case 3:
                        return true;
                    case 5:
                    case 6:
                    case 7:
                        return false;
                    default:
                        break;
                }
            }
            break;
        case RISCV_OPCODE_C1:
            break;
        case RISCV_OPCODE_C2:
            {
                uint8_t func = (ins >> 13) & 0x07;
                switch (func) {
                    case 0: // C.SLLI64
                        break;
                    case 1: // C.FLDSP
                    case 2: // C.LWSP
                    case 3: // C.LDSP
                        return true;
                    case 4: // C.JR C.MV C.EBREAK C.JALR C.ADD
                        break;
                    case 5: // C.FSDSP
                    case 6: // C.SWSP
                    case 7: // C.SDSP
                        return true;
                }
            }
            break;
        /* Use OOO's intrinsic register renaming to simulate vector instructions */
        case RISCV_OPCODE_VECTOR_LOAD:  // Vector loads
        case RISCV_OPCODE_VECTOR_STORE:
            return true;
        case RISCV_OPCODE_VECTOR_ARITH:  // Vector arithmetic operations
            break;
        default:
            break;
    }

    return false;
}

bool Decoder::decodeInstr(INS ins, DynUopVec& uops) {
    uint32_t initialUops = uops.size();
    bool inaccurate = false;
    uint8_t opcode = riscvInsOpCode(ins);
    uint8_t funct3 = riscvInsFunct3(ins);
    uint8_t funct7 = riscvInsFunct7(ins);

    Instr instr(ins);

    bool isLocked = false;
    // NOTE(dsm): IsAtomicUpdate == xchg or LockPrefix (xchg has in implicit lock prefix)
    if (riscvInsIsAtomic(ins)) {
        isLocked = true;
        emitFence(uops, 0); //serialize the initial load w.r.t. all prior stores
    }

    switch (opcode) {
        case RISCV_OPCODE_INTEGER:
            switch (funct3) {
                case 0x0: // ADD/SUB
                    if (funct7 == 0x00) {
                        // ADD
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    } else if (funct7 == 0x20) {
                        // SUB
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    } else if (funct7 == 0x01) {
                        // MUL (M extension)
                        emitMul(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                            riscvInsArithRs2(ins));
                    }
                    break;
                case 0x1: // SLL
                    if (funct7 == 0x00) {
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORT_0 | PORT_5);
                    } else if (funct7 == 0x01) {
                        // MULH (M extension)
                        emitMul(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                            riscvInsArithRs2(ins));
                    }
                    break;
                case 0x2: // SLT
                    if (funct7 == 0x00) {
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    } else if (funct7 == 0x01) {
                        // MULHSU (M extension)
                        emitMul(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                            riscvInsArithRs2(ins));
                    }
                    break;
                case 0x3: // SLTU
                    if (funct7 == 0x00) {
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    } else if (funct7 == 0x01) {
                        // MULHU (M extension)
                        emitMul(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                            riscvInsArithRs2(ins));
                    }
                    break;
                case 0x4: // XOR
                    if (funct7 == 0x00) {
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    } else if (funct7 == 0x01) {
                        // DIV (M extension)
                        emitDiv(uops, 64, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                            riscvInsArithRs2(ins));
                    }
                    break;
                case 0x5: // SRL/SRA
                    if (funct7 == 0x00) {
                        // SRL
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORT_0 | PORT_5);
                    } else if (funct7 == 0x20) {
                        // SRA
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORT_0 | PORT_5);
                    } else if (funct7 == 0x01) {
                        // DIVU (M extension)
                        emitDiv(uops, 64, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                            riscvInsArithRs2(ins));
                    }
                    break;
                case 0x6: // OR
                    if (funct7 == 0x00) {
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    } else if (funct7 == 0x01) {
                        // REM (M extension)
                        emitDiv(uops, 64, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                            riscvInsArithRs2(ins));
                    }
                    break;
                case 0x7: // AND
                    if (funct7 == 0x00) {
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    } else if (funct7 == 0x01) {
                        // REMU (M extension)
                        emitDiv(uops, 64, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                            riscvInsArithRs2(ins));
                    }
                    break;
                default:
                    inaccurate = true;
            }
            break;
        case RISCV_OPCODE_INTEGER_32:
            switch (funct3) {
                case 0x0: // ADDW/SUBW
                    if (funct7 == 0x00) {
                        // ADDW
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    } else if (funct7 == 0x20) {
                        // SUBW
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    } else if (funct7 == 0x01) {
                        // MULW (M extension)
                        emitMul(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                            riscvInsArithRs2(ins));
                    }
                    break;
                case 0x1: // SLLW
                    emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                        riscvInsArithRd(ins), 0, uops, 1, PORT_0 | PORT_5);
                    break;
                case 0x4: // DIVW (M extension)
                    emitDiv(uops, 32, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                        riscvInsArithRs2(ins));
                    break;
                case 0x5: // SRLW/SRAW
                    if (funct7 == 0x00) {
                        // SRLW
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORT_0 | PORT_5);
                    } else if (funct7 == 0x20) {
                        // SRAW
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORT_0 | PORT_5);
                    } else if (funct7 == 0x01) {
                        // DIVUW (M extension)
                        emitDiv(uops, 32, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                            riscvInsArithRs2(ins));
                    }
                    break;
                case 0x6: // REMW (M extension)
                case 0x7: // REMUW (M extension)
                    emitDiv(uops, 32, riscvInsArithRd(ins), riscvInsArithRs1(ins),
                        riscvInsArithRs2(ins));
                    break;
                default:
                    inaccurate = true;
            }
            break;
        case RISCV_OPCODE_INTEGER_IMM:
            switch (funct3) {
                case 0x0: // ADDI
                case 0x2: // SLTI
                case 0x3: // SLTIU
                case 0x4: // XORI
                case 0x6: // ORI
                case 0x7: // ANDI
                    emitExecUop(riscvInsArithRs1(ins), 0,
                        riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    break;
                case 0x1: // SLLI
                case 0x5: // SRLI/SRAI
                    emitExecUop(riscvInsArithRs1(ins), 0,
                        riscvInsArithRd(ins), 0, uops, 1, PORT_0 | PORT_5);
                    break;
                default:
                    inaccurate = true;
            }
            break;
        // RV64I I-type word instructions (opcode = 0x1B)
        case RISCV_OPCODE_INTEGER_IMM_32:
            switch (funct3) {
                case 0x0: // ADDIW
                    emitExecUop(riscvInsArithRs1(ins), 0,
                        riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
                    break;
                case 0x1: // SLLIW
                case 0x5: // SRLIW/SRAIW
                    emitExecUop(riscvInsArithRs1(ins), 0,
                        riscvInsArithRd(ins), 0, uops, 1, PORT_0 | PORT_5);
                    break;
                default:
                    inaccurate = true;
            }
            break;
        case RISCV_OPCODE_LOAD:
            emitLoad(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins));
            break;
        case RISCV_OPCODE_STORE:
            emitStore(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins));
            break;
        case RISCV_OPCODE_BRANCH:
        case RISCV_OPCODE_JAL:
            emitExecUop(0, 0,
                riscvInsArithRd(ins), 0, uops, 1, PORT_5);
            break;
        case RISCV_OPCODE_JALR:
            // All branches have the same uop profile in this model
            emitExecUop(riscvInsArithRs1(ins), 0,
                riscvInsArithRd(ins), 0, uops, 1, PORT_5);
            break;
        case RISCV_OPCODE_LUI:
            emitExecUop(0, 0,
                riscvInsArithRd(ins), 0, uops, 1, PORTS_015);
            break;
        case RISCV_OPCODE_AUIPC:
            emitExecUop(0, 0,
                riscvInsArithRd(ins), 0, uops, 1, PORT_1);
            break;
        case RISCV_OPCODE_ATOMIC:
            switch (funct3) {
                case 0x2: // Word operations
                case 0x3: // Double word operations
                    {
                        uint8_t amo_type = funct7 >> 2;
                        switch (amo_type) {
                            case 0x0: // AMOADD
                            case 0x4: // AMOXOR
                            case 0x8: // AMOOR
                            case 0xC: // AMOAND
                            case 0x10: // AMOMIN
                            case 0x14: // AMOMAX
                            case 0x18: // AMOMINU
                            case 0x1C: // AMOMAXU
                                {
                                    emitLoad(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins));
                                    emitExecUop(riscvInsArithRd(ins), riscvInsArithRs2(ins),
                                        riscvInsArithRd(ins), 0, uops, 3, PORTS_015);
                                    emitStore(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins));
                                }
                                break;
                            case 0x1: // AMOSWAP
                                /* TODO: check with uops */
                                emitXchg(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins), riscvInsArithRs2(ins));
                                break;
                            case 0x2: // LR (Load Reserved)
                                /* TODO: check with the register */
                                emitLoad(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins));
                                break;
                            case 0x3: // SC (Store Conditional)
                                emitStore(uops, riscvInsArithRs2(ins), riscvInsArithRs1(ins));
                                emitExecUop(0, 0,
                                        riscvInsArithRd(ins), 0, uops, 3, PORTS_015);
                                break;
                            default:
                                inaccurate = true;
                        }
                        break;
                    }
                default:
                    inaccurate = true;
            }
            break;
        case RISCV_OPCODE_SYSTEM:
            switch (funct3) {
                case 0x0: // ECALL, EBREAK, MRET, SRET, URET, WFI
                    emitExecUop(0, 0, 0, 0, uops, 1, PORTS_015);
                    break;
                case 0x1: // CSRRW
                case 0x2: // CSRRS
                case 0x3: // CSRRC
                case 0x5: // CSRRWI
                case 0x6: // CSRRSI
                case 0x7: // CSRRCI
                    // CSR operations are more complex
                    emitExecUop(0, 0, 0, 0, uops, 5, PORTS_015);
                    break;
                case 9: // SFENCE.VMA
                    /* TODO: check the latency value */
                    emitFence(uops, 25);
                    break;
                default:
                    inaccurate = true;
            }
            break;
        case RISCV_OPCODE_FENCE:
            switch (funct3) {
                case 0x0: // FENCE
                    emitFence(uops, 9);
                    break;
                case 0x1: // FENCE.I
                    emitFence(uops, 12);
                    break;
                default:
                    inaccurate = true;
            }
            break;
        case RISCV_OPCODE_MADD_FP: // FMADD.S, FMADD.D
        case RISCV_OPCODE_MSUB_FP: // FMSUB.S, FMSUB.D
        case RISCV_OPCODE_NMSUB_FP: // FNMSUB.S, FNMSUB.D
        case RISCV_OPCODE_NMADD_FP: // FNMADD.S, FNMADD.D
            {
                /* TODO: check fp register */
                uint8_t fmt = (ins >> 25) & 0x3;
                if (fmt == 0x0) { // Single precision
                    emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                        riscvInsArithRd(ins), 0, uops, 4, PORT_0);
                } else if (fmt == 0x1) { // Double precision
                    emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                        riscvInsArithRd(ins), 0, uops, 5, PORT_0);
                } else {
                    inaccurate = true;
                }
            }
            break;
        case RISCV_OPCODE_FP: // Floating point operations
            {
                uint32_t fmt = (ins >> 25) & 0x3;
                uint32_t fp_opcode = ins >> 27;
                
                switch (fp_opcode) {
                    case 0x0: // FADD.S/FADD.D
                    case 0x1: // FSUB.S/FSUB.D
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 3, PORT_1);
                        break;
                    case 0x2: // FMUL.S/FMUL.D
                        if (fmt == 0x0) { // FMUL.S
                            emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                                riscvInsArithRd(ins), 0, uops, 4, PORT_1);
                        } else if (fmt == 0x1) { // FMUL.D
                            emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                                riscvInsArithRd(ins), 0, uops, 5, PORT_1);
                        }
                        break;
                    case 0x3: // FDIV.S/FDIV.D
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 7, PORT_0, 6);
                        break;
                    case 0x4: // FSGNJ.S/FSGNJ.D, FSGNJN.S/FSGNJN.D, FSGNJX.S/FSGNJX.D
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 1, PORT_0 | PORT_5);
                        break;
                    case 0x5: // FMIN.S/FMIN.D, FMAX.S/FMAX.D
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 3, PORT_1);
                        break;
                    case 0x6: // FCVT.W.S/FCVT.W.D, FCVT.WU.S/FCVT.WU.D
                        emitExecUop(riscvInsArithRs1(ins), 0,
                            riscvInsArithRd(ins), 0, uops, 3 + 2, PORT_1);
                        break;
                    case 0x7: // FMV.X.W/FMV.X.D, FCLASS.S/FCLASS.D
                        emitExecUop(riscvInsArithRs1(ins), 0,
                        riscvInsArithRd(ins), 0, uops, 1 + 2, PORT_1);
                        break;
                    case 0x8: // FCMP.S/FCMP.D (FEQ, FLT, FLE)
                        emitExecUop(riscvInsArithRs1(ins), riscvInsArithRs2(ins),
                            riscvInsArithRd(ins), 0, uops, 3, PORT_1);
                        break;
                    case 0x9: // FCVT.S.W/FCVT.S.D, FCVT.S.WU/FCVT.D.WU
                        emitExecUop(riscvInsArithRs1(ins), 0,
                            riscvInsArithRd(ins), 0, uops, 3 + 2, PORT_1);
                        break;
                    case 0xA: // FMV.W.X/FMV.D.X
                    case 0xB: // FCVT.D.S/FCVT.S.D
                        emitExecUop(riscvInsArithRs1(ins), 0,
                            riscvInsArithRd(ins), 0, uops, 1 + 2, PORT_0);
                        break;
                    case 0xC: // FSQRT.S/FSQRT.D
                        emitExecUop(riscvInsArithRs1(ins), 0,
                            riscvInsArithRd(ins), 0, uops, 7, PORT_0);
                        break;
                    default:
                        inaccurate = true;
                }
            }
            break;
        /* To some floating compress instructions:
         * We will simply simulated it as a general purposed register
         * since the value does not matter
         */
        case RISCV_OPCODE_C0:
            {
                uint8_t func = (ins >> 13) & 0x07;
                uint8_t rd = riscvCompressedRegDecode((ins >> 2) & 0x7);
                uint8_t rs = riscvCompressedRegDecode((ins >> 7) & 0x7);
                switch(func) {
                    case 0:
                        emitExecUop(0, 0, rd, 0, uops, 1, PORTS_015);
                        break;
                    case 1:
                    case 2:
                    case 3:
                        emitLoad(uops, rd, rs);
                        break;
                    case 5:
                    case 6:
                    case 7:
                        emitStore(uops, rs, rd);
                        break;
                    default:
                        inaccurate = true;
                }
            }
            break;
        case RISCV_OPCODE_C1:
            {
                uint8_t func = (ins >> 13) & 0x07;
                switch (func) {
                    case 0: // C.ADDI
                    case 1: // C.ADDIW
                        {
                            uint8_t rsrd = (ins >> 7) & 0x1f;
                            emitExecUop(rsrd, 0, rsrd, 0, uops, 1, PORTS_015);
                        }
                        break;
                    case 2: // C.Li
                    case 3: // C.ADDI16SP C.LUI
                        {
                            uint8_t rsrd = (ins >> 7) & 0x1f;
                            emitExecUop(0, 0, rsrd, 0, uops, 1, PORTS_015);
                        }
                        break;
                    case 4: // Shift and Arith
                        {
                            uint8_t rsrd = riscvCompressedRegDecode((ins >> 7) & 0x07);
                            uint8_t subFunc = (ins >> 10) & 0x03;
                            if (subFunc != 3) {
                                emitExecUop(rsrd, 0, rsrd, 0, uops, 1, PORT_0 | PORT_5);
                            } else {
                                uint8_t rs2 = riscvCompressedRegDecode((ins >> 2) & 0x07);
                                emitExecUop(rsrd, rs2, rsrd, 0, uops, 1, PORTS_015);
                            }
                        }
                        break;
                    case 5: // C.J
                        emitExecUop(0, 0, 1, 0, uops, 1, PORT_5);
                        break;
                    case 6: // C.BEQZ
                    case 7: // C.BNEZ
                        {
                            uint8_t rs = riscvCompressedRegDecode((ins >> 7) & 0x07);
                            emitExecUop(rs, 0, 0, 0, uops, 1, PORT_5);
                        }
                        break;
                }
            }
            break;
        case RISCV_OPCODE_C2:
            {
                uint8_t func = (ins >> 13) & 0x07;
                uint8_t rsrd = (ins >> 7) & 0x1f;
                uint8_t rs2 = (ins >> 2) & 0x1f;
                switch (func) {
                    case 0: // C.SLLI64
                        emitExecUop(rsrd, 0, rsrd, 0, uops, 1, PORT_0 | PORT_5);
                        break;
                    case 1: // C.FLDSP
                    case 2: // C.LWSP
                    case 3: // C.LDSP
                        emitLoad(uops, rsrd, 2);
                        break;
                    case 4: // C.JR C.MV C.EBREAK C.JALR C.ADD
                        {
                            uint8_t funct1 = (ins >> 12) & 0x01;
                            if (funct1 == 0) {
                                if (rs2 == 0) {
                                    emitExecUop(0, 0, rsrd, 0, uops, 1, PORT_5);
                                } else {
                                    emitExecUop(rs2, 0, rsrd, 0, uops, 1, PORT_5);
                                }
                            } else {
                                if (rsrd == 0 && rs2 == 0) {
                                    /* EBREAK */
                                    emitExecUop(0, 0, 0, 0, uops, 1, PORTS_015);
                                } else if (rsrd != 0 && rs2 == 0) {
                                    /* C.JALR */
                                    emitExecUop(rsrd, 0, 1, 0, uops, 1, PORT_5);
                                } else {
                                    /* C.ADD */
                                    emitExecUop(rsrd, rs2, rsrd, 0, uops, 1, PORTS_015);
                                }
                            }
                        }
                        break;
                    case 5: // C.FSDSP
                    case 6: // C.SWSP
                    case 7: // C.SDSP
                        emitStore(uops, 2, rs2);
                        break;
                }
            }
            break;
        /* Use OOO's intrinsic register renaming to simulate vector instructions */
        case RISCV_OPCODE_VECTOR_LOAD:  // Vector loads
        case RISCV_OPCODE_VECTOR_STORE:
            {
                if (funct3 != 0) {
                    if (opcode == RISCV_OPCODE_VECTOR_LOAD) {
                        emitLoad(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins));
                    } else {
                        emitStore(uops, riscvInsArithRd(ins), riscvInsArithRs1(ins));
                    }
                } else {
                    /* Loads and stores of these instructions will be provided
                     * by the frontend
                     */
                }
            }
            break;
        case RISCV_OPCODE_VECTOR_ARITH:  // Vector arithmetic operations
            {
                // uint32_t vectorOp = INS_VectorOp(ins);
                // uint32_t width = INS_VectorWidth(ins);
                // uint32_t elements = INS_VectorElements(ins);
                uint32_t width = 32, elements = 16;
                
                // Approximate vector operations based on width and element count
                uint32_t baseLatency = 1;
                uint8_t port = PORTS_015;
                
                /* TODO: check this */
                // Adjust latency based on operation type
                // switch (vectorOp) {
                //     case RV_VOP_ADD:
                //     case RV_VOP_SUB:
                //     case RV_VOP_AND:
                //     case RV_VOP_OR:
                //     case RV_VOP_XOR:
                //         baseLatency = 1;
                //         break;
                //     case RV_VOP_MUL:
                //         baseLatency = 4;
                //         port = PORT_0;
                //         break;
                //     case RV_VOP_DIV:
                //         baseLatency = 20;
                //         port = PORT_0;
                //         break;
                //     case RV_VOP_FP_ADD:
                //     case RV_VOP_FP_SUB:
                //         baseLatency = 3;
                //         port = PORT_1;
                //         break;
                //     case RV_VOP_FP_MUL:
                //         baseLatency = 4;
                //         port = PORT_0;
                //         break;
                //     case RV_VOP_FP_DIV:
                //         baseLatency = 10;
                //         port = PORT_0;
                //         break;
                //     default:
                //         baseLatency = 1;
                // }

                baseLatency = 8;
                port = PORT_0;
                
                // Emit vector operations based on vector length
                uint32_t numUops = (width * elements + 63) / 64;  // Rough approximation
                numUops = numUops == 0 ? 1 : numUops;  // At least one uop
                
                for (uint32_t i = 0; i < numUops; i++) {
                    emitExecUop(0, 0, 0, 0, uops, baseLatency, port);
                }
            }
            break;
            
        default:
            inaccurate = true;
            // Try to produce something approximate
            emitExecUop(0, 0, 0, 0, uops, 1, PORTS_015);
    }

    //NOTE: REP instructions are unrolled by PIN, so they are accurately simulated (they are treated as predicated in Pin)
    //See section "Optimizing Instrumentation of REP Prefixed Instructions" on the Pin manual

    //Add ld/st fence to all locked instructions
    if (isLocked) {
        //inaccurate = true; //this is now fairly accurate
        emitFence(uops, 9); //locked ops introduce an additional uop and cache locking takes 14 cycles/instr per the perf counters; latencies match with 9 cycles of fence latency
    }

    assert(uops.size() - initialUops < MAX_UOPS_PER_INSTR);
    //assert_msg(uops.size() - initialUops < MAX_UOPS_PER_INSTR, "%ld -> %ld uops", initialUops, uops.size());
    return inaccurate;
}

// See Agner Fog's uarch doc, macro-op fusion for Core 2 / Nehalem
bool Decoder::canFuse(INS ins) {
    uint8_t opcode = riscvInsOpCode(ins);
    switch (opcode) {
        case RISCV_OPCODE_BRANCH:
        case RISCV_OPCODE_JAL:
            return true;
        case RISCV_OPCODE_JALR:
        default:
            return false;
    }
}

BblInfo* Decoder::decodeBbl(struct BasicBlock &bbl, bool oooDecoding) {
    auto bytes = bbl.codeBytes;
    BblInfo* bblInfo;

    bbl.resetProgramIndex();
    //Gather some info about instructions needed to model decode stalls
    std::vector<ADDRINT> instrAddr;
    std::vector<uint32_t> instrBytes;
    std::vector<uint32_t> instrUops;
    std::vector<INS> instrDesc;

    //Decode BBL
    uint32_t approxInstrs = 0;
    DynUopVec uopVec;
    //Decode
    size_t instIndex = 0;
    uint8_t instLength = 0;
    for (INS ins = bbl.getHeadInstruction(&instIndex, &instLength); !bbl.endOfBlock();
            ins = bbl.getHeadInstruction(&instIndex, &instLength)) {
        bool inaccurate = false;
        uint32_t prevUops = uopVec.size();
        inaccurate = Decoder::decodeInstr(ins, uopVec);

        instrAddr.push_back(bbl.virtualPc + instIndex);
        instrBytes.push_back(instLength);
        instrUops.push_back(uopVec.size() - prevUops);
        instrDesc.push_back(ins);

        if (inaccurate) {
            approxInstrs++;
        }
        if (Decoder::canFuse(ins)) {
            break;
        }
    }

    if (oooDecoding) {
        //Instr predecoder and decode stage modeling; we assume clean slate between BBLs, which is typical because
        //optimizing compilers 16B-align most branch targets (and if it doesn't happen, the error introduced is fairly small)

        //1. Predecoding
        std::vector<uint32_t> predecCycle;
        uint32_t pcyc = 0;
        uint32_t psz = 0;
        uint32_t pcnt = 0;
        uint32_t pblk = 0;

        ADDRINT startAddr = bbl.virtualPc & ~0xfUL;

        for (uint32_t i = 0; i < instrDesc.size(); i++) {
            ADDRINT addr = instrAddr[i];
            uint32_t bytes = instrBytes[i];
            uint32_t block = (addr - startAddr) >> 4;
            psz += bytes;
            pcnt++;
            if (psz > 16 /*leftover*/|| pcnt > 6 /*max predecs*/|| block > pblk /*block switch*/) {
                psz = bytes;
                pcnt = 1;
                pblk = block;
                pcyc++;
            }

            //Length-changing prefix introduce a 6-cycle penalty regardless;
            //In 64-bit mode, only operand size prefixes are LCPs; addr size prefixes are fine
            // UPDATE (dsm): This was introducing significant errors in some benchmarks (e.g., astar)
            // Turns out, only SOME LCPs (false LCPs) cause this delay
            // see http://www.jaist.ac.jp/iscenter-new/mpc/altix/altixdata/opt/intel/vtune/doc/users_guide/mergedProjects/analyzer_ec/mergedProjects/reference_olh/pentiumm_hh/pentiummy_hh/lipsmy/instructions_that_require_slow_decoding.htm
            // At this point I'm going to assume that gcc is smart enough to not produce these
            //if (INS_OperandSizePrefix(ins)) pcyc += 6;

            predecCycle[i] = pcyc;
            //info("PREDEC %2d: 0x%08lx %2d %d %d %d", i, instrAddr[i], instrBytes[i], instrUops[i], block, predecCycle[i]);
        }

        //2. Decoding
        //4-1-1-1 rules: Small decoders can only take instructions that produce 1 uop AND are at most 7 bytes long
        uint32_t uopIdx = 0;

        uint32_t dcyc = 0;
        uint32_t dsimple = 0;
        uint32_t dcomplex = 0;

        for (uint32_t i = 0; i < instrDesc.size(); i++) {
            if (instrUops[i] == 0) continue; //fused branch

            uint32_t pcyc = predecCycle[i];
            if (pcyc > dcyc) {
                dcyc = pcyc;
                dsimple = 0;
                dcomplex = 0;
            }

            bool simple = (instrUops[i] == 1) && (instrBytes[i] < 8);

            if ((simple && dsimple + dcomplex == 4) || (!simple && dcomplex == 1)) { // Do: (!simple /*&& dcomplex == 1*/) to be conservative?
                dcyc++;
                dsimple = 0;
                dcomplex = 0;
            }

            if (simple) dsimple++;
            else dcomplex++;

            //info("   DEC %2d: 0x%08lx %2d %d %d %d (%d %d)", i, instrAddr[i], instrBytes[i], instrUops[i], simple, dcyc, dcomplex, dsimple);

            for (uint32_t j = 0; j < instrUops[i]; j++) {
                uopVec[uopIdx + j].decCycle = dcyc;
            }

            uopIdx += instrUops[i];
        }

        assert(uopIdx == uopVec.size());

        //Allocate
        uint32_t objBytes = offsetof(BblInfo, oooBbl) + DynBbl::bytes(uopVec.size());
        bblInfo = static_cast<BblInfo*>(gm_malloc(objBytes));  // can't use type-safe interface

        //Initialize ooo part
        DynBbl& dynBbl = bblInfo->oooBbl[0];
        dynBbl.addr = bbl.virtualPc;
        dynBbl.uops = uopVec.size();
        dynBbl.approxInstrs = approxInstrs;
        for (uint32_t i = 0; i < dynBbl.uops; i++) dynBbl.uop[i] = uopVec[i];

#ifdef BBL_PROFILING
        futex_lock(&bblIdxLock);
        dynBbl.bblIdx = bblIdx++;
        assert(dynBbl.bblIdx < MAX_BBLS);
        if (approxInstrs) {
            bblApproxOpcodes[dynBbl.bblIdx] = new std::vector<uint32_t>(approxOpcodes);  // copy
        }
        //info("DECODED BBL IDX %d", bblIdx);

        futex_unlock(&bblIdxLock);
#endif
    } else {
        bblInfo = gm_malloc<BblInfo>();
    }

    //Initialize generic part
    bblInfo->instrs = instrDesc.size();
    bblInfo->bytes = bytes;
    std::cout << "Instructions: " << bblInfo->instrs << std::endl;

    return bblInfo;
}


#ifdef BBL_PROFILING
void Decoder::profileBbl(uint64_t bblIdx) {
    assert(bblIdx < MAX_BBLS);
    __sync_fetch_and_add(&bblCount[bblIdx], 1);
}

void Decoder::dumpBblProfile() {
    uint32_t numOpcodes = xed_iform_enum_t_last() + 1;
    uint64_t approxOpcodeCount[numOpcodes];
    for (uint32_t i = 0; i < numOpcodes; i++) approxOpcodeCount[i] = 0;
    for (uint32_t i = 0; i < bblIdx; i++) {
        if (bblApproxOpcodes[i]) for (uint32_t& j : *bblApproxOpcodes[i]) approxOpcodeCount[j] += bblCount[i];
    }

    std::ofstream out("approx_instrs.stats");
    out << std::setw(16) << "Category" << std::setw(16) << "Iclass" << std::setw(32) << "Iform" << std::setw(16) << "Count" << std::endl;
    for (uint32_t i = 0; i < numOpcodes; i++) {
        if (approxOpcodeCount[i]) {
            //out << xed_iclass_enum_t2str((xed_iclass_enum_t)i) << "\t " << approxOpcodeCount[i] << std::endl;
            xed_iform_enum_t iform = (xed_iform_enum_t)i;
            xed_category_enum_t cat = xed_iform_to_category(iform);
            xed_iclass_enum_t iclass = xed_iform_to_iclass(iform);

            out << std::setw(16) << xed_category_enum_t2str(cat) << std::setw(16) << xed_iclass_enum_t2str(iclass) << std::setw(32) << xed_iform_enum_t2str(iform) << std::setw(16) << approxOpcodeCount[i] << std::endl;
        }
    }

    //Uncomment to dump a bbl profile
    //for (uint32_t i = 0; i < bblIdx; i++) out << std::setw(8) << i << std::setw(8) <<  bblCount[i] << std::endl;

    out.close();
}

#endif

