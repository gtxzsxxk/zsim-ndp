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

#ifndef VIRT_COMMON_H_
#define VIRT_COMMON_H_

// Typedefs and common functions for Virt implementation
// This is internal to virt, and should only be included withing virt/ files

#include "galloc.h"
#include "log.h"
#include "virt/virt.h"

struct PrePatchArgs {
    uint32_t tid;
    CONTEXT* ctxt;
    SYSCALL_STANDARD std;
    const char* patchRoot;
    bool isNopThread;
};

struct PostPatchArgs {
    uint32_t tid;
    CONTEXT* ctxt;
    SYSCALL_STANDARD std;
};

// Define our own wrapper for lambda closures instead of std::function.
class PostPatchFn {
private:
    class LambdaWrapperBase : public GlobAlloc {
    public:
        virtual ~LambdaWrapperBase() = default;
        virtual PostPatchAction invoke(PostPatchArgs&) = 0;
        virtual LambdaWrapperBase* clone() const = 0;
    };

    template<typename Lambda>
    class LambdaWrapper : public LambdaWrapperBase {
    private:
        Lambda l;
    public:
        LambdaWrapper(const Lambda& _l) : l(_l) {}
        ~LambdaWrapper() override = default;
        PostPatchAction invoke(PostPatchArgs& args) { return l(args); }
        LambdaWrapperBase* clone() const { return new LambdaWrapper<Lambda>(l); }
    };

private:
    LambdaWrapperBase* w;

public:
    template<typename Lambda>
    PostPatchFn(Lambda l) {
        w = new LambdaWrapper<Lambda>(l);
    }

    PostPatchFn() {
        w = nullptr;
    }

    PostPatchFn(const PostPatchFn& other) {
        w = other.w ? other.w->clone() : nullptr;
    }

    PostPatchFn& operator=(const PostPatchFn& other) {
        w = other.w ? other.w->clone() : nullptr;
        return *this;
    }

    PostPatchFn(PostPatchFn&& other) {
        delete w;
        w = other.w;
        other.w = nullptr;
    }

    PostPatchFn& operator=(PostPatchFn&& other) {
        delete w;
        w = other.w;
        other.w = nullptr;
        return *this;
    }

    ~PostPatchFn() {
        delete w;
        w = nullptr;
    }

    PostPatchAction operator()(PostPatchArgs args) {
        return w ? w->invoke(args) : PPA_NOTHING;
    }
};

typedef PostPatchFn (*PrePatchFn)(PrePatchArgs);

extern const PostPatchFn NullPostPatch; // defined in virt.cpp

// PIN_SafeCopy wrapper. We expect the default thing to be correct access
template<typename T>
static inline bool safeCopy(const T* src, T* dst, const char* file = __FILE__, int line = __LINE__) {
    size_t copiedBytes = PIN_SafeCopy(dst, src, sizeof(T));
    if (copiedBytes != sizeof(T)) {
        warn("[%d] %s:%d Failed app<->tool copy (%ld/%ld bytes copied)", PIN_ThreadId(), file, line, copiedBytes, sizeof(T));
        return false;
    }
    return true;
}

#endif  // VIRT_COMMON_H_
