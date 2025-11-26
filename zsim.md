# zsim分析

## 启动流程

### zsim harness

编译zsim项目会产生两个二进制，一个是libzsim.so，一个是zsim。zsim的本体是zsim_harness.cpp，而与PIN有关的zsim.cpp会被编到libzsim.so内。

zsim_harness的main函数内，根据第一个参数得到configFile的路径，然后设置每个signal的处理，几乎只要是signal就直接重开，把整个进程树都干掉。

然后根据sim.gmMBytes申请内存，然后使用**PinCmd**类来加载libzsim.so，生成目标程序正确的args，得到总共的进程数（也就是需要仿真多少个程序，在zsim.cfg中用process0、process1进行配置），然后创建进程。

进入创建进程的函数（LaunchProcess）后，会调用fork创建新的进程，parent进程会直接摸鱼返回，而child进程就要调用execvp进入目标程序，顺便进入zsim.cpp的main函数。

这个时候parent进程就会开始摸鱼，等待子进程结束。作为外部观察者，监控模拟是否“卡住”。wait() 所有退出的子进程，防止它们变“僵尸”。定期打印心跳，并在模拟器崩溃时提供一些线索。

### zinfo

**GlobSimInfo**是一个基于共享内存的进程间状态管理类。

### libzsim.so

进入到libzsim.so后，将执行zsim.cpp的main函数。

首先执行了PIN_InitSymbols和PIN_Init，初始化PIN。然后调用PIN_AddInternalExceptionHandler设置InternalExceptionHandler函数为异常handler，本质上就是backtrace然后结束进程。

然后会获取目前属于第几个进程，如果是第0个进程，就开始执行SimInit函数。

#### SimInit

SimInit函数会初始化zinfo，配置好仿真的核心数，设置好sim.domains、sim.contentionThreads等选项，初始化**ContentionSim**类，

然后配置sim.phaseLength、sim.statsPhaseInterval、sys.frequency等。

然后初始化一个**EventQueue**类，如果没有被配置成traceDriven模式，那么就初始化**Scheduler**类。

调用InitGlobalStats初始化全局状态。

初始化**AggregateStat**类，初始化**PortVirtualizer**类。

然后调用CreateProcessTree创建进程树。调用InitNUMA设置和验证NUMA模拟环境，然后创建**NUMAMap**类。

调用InitSystem初始化cache、core和memory controller。会把这些的hierarchy组合成一个map和parent、child的关系，InitSystem 函数 new 出了一系列核心类，主要包括：SimpleCore、TimingCore 或 OOOCore（CPU 核心），BaseCache 的各种子类（缓存），MemInterconnect、MemRouter 和 MemInterconnectInterface（总线/互连），以及 BuildMemoryController 返回的 MemObject（内存条）。这些 new 好的对象并没有一个统一的“全局列表”；相反，它们通过 setParents() 和 setChildren() 方法被“焊接”在了一起。最终，所有的 CPU 核心 指针被存放在全局的 zinfo->cores 数组中，而缓存、总线和内存则通过彼此的父/子指针，构成了一个从核心（Core）直达内存（Memory）的完整、可遍历的层级树。

如果定义了，就执行zinfo->sched->initStats。new **ProcessStats**类，new **ProcStats**类，new **VectorCounter**类作为zinfo->profHeartbeats的值，处理一些杂项后执行zinfo->contentionSim->postInit，并设置gm_set_glob_ptr(zinfo)，即完成SimInit。

SimInit调用结束后，初始化fPtrs和cids。fPtrs是所有仿真thread的函数指针集合。每一个thread对应了自己的loadPtr、storePtr、bblPtr、branchPtr、predLoadPtr、predStorePtr，这些是zsim中对不同仿真数据的处理函数。cids是物理上的标识符，标识代表zsim模拟的硬件CPU核心（core）。tid(Thread ID): 逻辑标识符。代表正在模拟的应用程序中的线程。zsim中还有一个gid，就是一个32位的整数，它的高16位是pid，低16位是tid。

然后开始调用VirtCaptureClocks设置虚拟化的**ClockDomainInfo**类。

#### FFIInit

然后调用FFIInit函数在进程start时初始化Fast-Forward Intervals。这个函数读取ffiPoints列表（例如 [1M, 9M]）。设置第一个指令限制 ffiInstrsLimit = 1000000。此时，模拟器处于NFF (详细模拟) 模式。

调用FFITrackNFFInterval()。这个函数“设置一个闹钟”。它使用makeAdaptiveEvent创建一个事件，插入到zinfo->eventQueue中。这个事件将在 ffiInstrsLimit这么多（即100万条）详细指令被执行后触发。触发的是ffiFire lambda(切换到 FF)：当这100万条详细指令执行完毕，"闹钟"响起，ffiFire这个lambda匿名函数被执行。它打印 "Entering fast-forward" 并调用 zinfo->procArray[p]->enterFastForward()。模拟器切换到FF(快进)模式。FFIEntryBasicBlock作为过渡函数被调用一次，它会调用FFIAdvance()。

然后执行VirtInit函数。这个函数主要就是patch系统调用，patch了以下部分：

#### VirtInit

初始化zsim的系统调用虚拟化功能。

1. 📁 文件系统 (fs.cpp) Syscalls: SYS_open, SYS_openat。用途： 拦截文件打开请求。这是为了提供一个虚拟文件系统（patchRoot）。当程序尝试读取 /sys/devices/system/node（NUMA 拓扑）或 /proc/cpuinfo（CPU 信息）时，zsim 会拦截这个请求，并给它返回一个伪造的、描述模拟系统拓扑的文件。

2. 🌐 端口/网络 (ports.cpp) Syscalls: SYS_bind, SYS_getsockname, SYS_connect。用途： 虚拟化网络端口。这允许多个 zsim 模拟的进程在模拟的网络上互相通信，而不会占用或干扰宿主机的真实网络端口。

3. 🖥️ CPU 虚拟化 (cpu.cpp) Syscalls: SYS_getcpu, SYS_sched_getaffinity, SYS_sched_setaffinity。用途： 向程序“谎报”CPU 信息。当程序询问“我在哪个CPU上运行？”（getcpu）或“我允许在哪些CPU上运行？”（getaffinity）时，zsim 会拦截它，并根据 Scheduler 的调度（gid -> cid 的映射）返回模拟的 CPU 核心 ID（cid），而不是宿主机的 CPU ID。

4. 🧠 NUMA 虚拟化 (numa.cpp) Syscalls: SYS_get_mempolicy, SYS_set_mempolicy, SYS_mbind, SYS_munmap, SYS_mremap 等。用途： 这是 NUMAMap 机制的核心。zsim 必须拦截所有管理内存策略的调用。set_mempolicy: 当程序设置内存策略（如“本地优先”或“交错”）时，zsim 拦截它，并将这个策略保存到 NUMAMap 的 threadPolicy 中。munmap: 当程序释放内存时，zsim 拦截它，以便更新 NUMAMap 中的 PageMap（页 -> 节点）映射。

5. 🛑 线程控制 (control.cpp) Syscalls: SYS_exit_group。用途： 优雅地终止模拟。当被模拟的程序（的主线程）退出时，zsim 必须捕获这个事件，以便停止所有模拟组件、刷新所有统计数据并干净地退出，而不是让整个 Pin 工具崩溃。

6. ⏰ 时间与超时 (time.cpp, timeout.cpp) Syscalls: SYS_gettimeofday, SYS_time, SYS_nanosleep, SYS_futex, SYS_poll 等。用途： 时间虚拟化。这是模拟器最关键的补丁之一。获取时间： 当程序询问“现在几点了？”（gettimeofday），zsim 会返回模拟时间（即 zinfo->globPhaseCycles），而不是真实的挂钟时间。阻塞/睡眠： 当程序调用 nanosleep 或 futex（等待锁）时，它本意是“阻塞”自己。如果 zsim 让宿主机 OS 真的阻塞它，整个模拟都会卡住。zsim 会拦截这个调用，在 Scheduler 中将该线程设为 SLEEPING 或 BLOCKED 状态，然后继续推进模拟时间，直到满足唤醒条件。

#### VdsoInit

在调用完VirtInit函数后，就会开始调用VdsoInit函数。VirtInit 负责拦截普通的系统调用（那些会陷入内核的），而 VdsoInit 负责拦截走捷径的系统调用（那些完全在用户态完成的）。主要针对clock_gettime、gettimeofday、time、getcpu。

#### 插桩

在初始化完虚拟化后，就要进行插桩了，插桩的部分如下：

1. 核心指令流插桩。使用TRACE_AddInstrumentFunction函数，每当 Pin 发现一个新的 Trace，它就会调用 zsim 的 Trace() 函数。
2. 线程生命周期管理。使用PIN_AddThreadStartFunction函数和PIN_AddThreadFiniFunction函数。当目标程序 pthread_create 时，zsim 需要在自己的 Scheduler 中注册这个新线程，分配一个 ThreadInfo，并给它分配一个模拟核心 (cid)。Fini: 当线程退出时，zsim 需要回收资源，并在 Scheduler 中注销它。实现的函数是ThreadStart和ThreadFini。
3. 系统调用拦截。使用PIN_AddSyscallEntryFunction函数和PIN_AddSyscallExitFunction函数。调用的函数为SyscallEnter和SyscallExit。使用PIN_AddContextChangeFunction函数当发生非正常的控制流改变（主要是信号 Signal 处理）时调用ContextChange函数。
4. 进程与多任务。使用PIN_AddFiniFunction函数在目标程序彻底结束（exit）时调用Fini函数。Fini函数又会接着调用SimEnd函数执行真正的结束任务。同时需要处理进程树的模拟。使用PIN_AddFollowChildProcessFunction指定FollowChild（告诉 Pin，如果当前程序调用了 exec 启动新程序，请继续跟踪新程序，把PinTool注入进去）。然后PIN_AddForkFunction设置调用BeforeFork（赋值静态变量forkedChildNode为procTreeNode->getNextChild()），AfterForkInParent（forkedChildNode设置为nullptr）和AfterForkInChild（调用procTreeNode->notifyStart()，输出新进程的信息，并且初始这个新PIN进程的fPtrs为joinPtrs、cids为UNINITIALIZED_CID，并且让子进程也开始执行FFThread）。然后主进程的代码创建了一个新的线程并执行FFThread函数，然后自己通过PIN_StartProgram函数不再返回。所以这里其实模拟器程序本身最终运行的都是FFThread函数，不管是主进程还是fork后的进程，都需要FFThread函数进行仿真。

### Trace

区分两个时间点：

1. 插桩时 (Instrumentation Time)：即 Trace() 函数执行的时候。这是 Pin 在即时编译（JIT）代码，它决定了“要插入什么代码”。

2. 分析/执行时 (Analysis/Execution Time)：即目标程序真正运行的时候。此时，被插入的那些函数（如 IndirectBasicBlock）才会被真正调用。

#### 1. 总入口

当 Pin 发现一段新的代码轨迹（Trace，通常是一串基本块 Basic Block）时调用。它的任务是遍历这段代码，插入 zsim 的回调函数。

代码首先遍历 Trace 中的每一个基本块（BBL）。前提条件： 如果当前处于“快进模式”(FastForward) 且不需要重插桩，则跳过此步（为了性能）。

- Decoder::decodeBbl(bbl, ...)：
    - 干什么：这是一个静态分析过程。它分析这个 BBL 里有多少指令、有多少内存操作、指令间的依赖关系等。
    - 产出：生成一个 BblInfo 结构体，里面包含了模拟这个 BBL 所需的所有元数据（指令数、字节数等）。
- BBL_InsertCall(..., IndirectBasicBlock, ...)：
    - 干什么：在 BBL 的开头插入一个调用。
    - 插入了谁：IndirectBasicBlock。
    - 参数：传入了 BblInfo 指针。
    - 意义：当程序运行到这个基本块时，zsim 会先获得控制权，拿到这个块的元数据，然后算出这个块消耗了多少周期（Cycle），推进模拟时间。

#### 2. 指令 (Instruction) 级插桩

接着，代码再次遍历 BBL，并深入遍历其中的每一条指令（INS）。调用：Instruction(ins)。职责：决定对当前这条具体的指令“做点什么”。

- 内存访问插桩 (Memory Access)
    - 判断：通过 INS_IsMemoryRead / INS_IsMemoryWrite 检查指令是否读写内存。
    - 插入：
        - 如果是读：插入 IndirectLoadSingle。
        - 如果是写：插入 IndirectStoreSingle。
    - 参数：传入 IARG_MEMORYREAD_EA (有效地址)。
    - 意义：这是 Cache 模拟的基础。zsim 需要拦截每一次内存访问的地址，扔给 Cache 层次结构去判断是 Hit 还是 Miss。
- 分支预测插桩 (Branch Prediction)
    - 判断：INS_Category(ins) == XED_CATEGORY_COND_BR（条件跳转）。
    - 插入：IndirectRecordBranch。
    - 意义：将分支的跳转方向（Taken/Not Taken）和目标地址传给 CPU 模型的分支预测器 (Branch Predictor)，用来更新预测历史并计算预测失败的惩罚。

##### 虚拟化与魔法指令 (Virtualization & Magic)

这是 zsim 能够“欺骗”目标程序的关键。

- Magic Ops (xchg %rcx, %rcx)：
    - 插入：HandleMagicOp。
    - 功能：这是一个“后门”。被模拟的程序可以通过这条特殊的汇编指令告诉 zsim：“开始感兴趣区域(ROI)”、“心跳”、“注册线程”等。这实现了 Guest 和 Host 的通信。
- CPUID (FakeCPUIDPre/Post)：
    - 插入：在 CPUID 指令的前后插入。
    - 功能：CPUID 指令通常返回宿主机的 CPU 信息。zsim 必须拦截它，篡改寄存器（EAX, EBX 等）的值，让程序以为自己运行在 zsim 配置的那个 64 核 CPU 上，而不是你的笔记本电脑上。代码中可以看到它计算 apicId 和 siblings 的逻辑。
- RDTSC (FakeRDTSCPost)：
    - 插入：在 RDTSC 指令后插入。
    - 功能：RDTSC 读取硬件的时间戳计数器。zsim 必须拦截并覆盖它的返回值（EAX:EDX），填入 zinfo->globPhaseCycles（模拟时间）。如果这里不拦截，程序读到的是真实世界的飞快流逝的时间，模拟结果就会完全错误（Time Leakage）。

#### 3. Indirect... 系列函数与 fPtrs：多态的实现

fPtrs (Function Pointers)：这是一个数组，每个线程 (tid) 对应一个 InstrFuncPtrs 结构体，里面存着 loadPtr, storePtr, bblPtr 等函数指针。

- 当线程处于 Detailed Simulation (NFF) 模式时，fPtrs[tid] 指向真正的模拟函数（如 SimLoad）。
- 当线程处于 Fast Forward (FF) 模式时，fPtrs[tid] 指向空函数（NopLoad）。
- 当线程处于 Null/Blocked 状态时，它指向另一组函数。

### FFThread

这个机制的设计初衷是为了解决 Intel Pin 在多线程环境下处理信号（Signal）不稳定的问题，因此作者选择了一个“笨办法”：用一个线程死循环等待外部触发。

1. FFThread 的核心逻辑：无限等待循环
    - 角色：这是一个“守护线程”，它不参与模拟，只负责监听开关请求。
    - 启动：在 main 函数中通过 PIN_SpawnInternalThread 启动。
        1. 初始化锁：ffToggleLocks 是通信的媒介。线程先锁住它，然后试图再次加锁（见下文），这样它就会阻塞。外部触发器（例如另一个工具 fftoggle）通过解锁 (unlock) 这个 mutex 来“捅”一下 zsim，告诉它“该切换模式了”。
        2. 进入循环等待触发。`bool locked = futex_trylock_nospin_timeout(..., 5*BILLION);`，线程在这里阻塞。它尝试去锁那个已经被自己锁住的锁（或者等待别人解锁它）。超时机制 (5秒)：它每 5 秒醒来一次，不是为了切换，而是检查 zinfo->terminationConditionMet（模拟是否结束了？），以便优雅退出。被唤醒：如果 locked == true，说明有人在外面解开了这把锁。这意味着收到了切换信号。
        3. 处理切换请求。拿到全局锁 zinfo->ffLock 保护状态。判断当前状态：情况 A：正在快进 (FF) -> 切回详细模拟。情况 B：正在详细模拟 (NFF) -> 切入快进。
2. 退出快进 (ExitFastForward) - "简单模式"
    - 如果当前是快进模式，切换回详细模拟是比较安全的，可以立即执行。
    - 调用 ExitFastForward()
        1. VirtCaptureClocks(true)：因为快进时使用的是“假”时间，切回详细模拟前，必须把虚拟时间（TSC/Cycle）同步对齐，确保模拟的时间连续性。
        2. procTreeNode->exitFastForward()：修改内存中的状态标志 inFastForward = false。原子更新全局计数器。
        3. __sync_synchronize()：内存屏障，确保标志位变化对所有线程可见。
        4. PIN_RemoveInstrumentation() (如果配置了 ffReinstrument)： 这会告诉 Pin：“丢弃当前所有的 JIT 代码缓存”。后果：Pin 被迫重新插桩。此时 inFastForward 变成了 false，所以再次调用 Trace() 函数时，会插入 IndirectBasicBlock 和 Load/Store 的回调，从而开启详细模拟。
3. 进入快进 (EnterFastForward) - "困难模式"
    - 如果当前正在进行详细模拟，我们不能立即切换。因为模拟器是以 Phase (阶段) 为单位运行的（例如每 10000 个周期同步一次）。如果在阶段中间突然切断详细模拟，会导致不同核心的时间不同步。因此，FFThread 必须等待当前 Phase 结束。
    - 调用链与握手流程：
        1. `zinfo->eventQueue->insert(syncEv);`,FFThread 往全局事件队列里扔了一个 SyncEvent。这个事件会在下一个 Phase 结束时被主模拟线程处理。
        2. `syncEv->wait();`，释放锁并等待 (wait)。
        3. 主模拟线程 (Main Simulation Thread) 的工作（此时 FFThread 在睡觉）:
            - 它跑完当前的 Phase。
            - 处理事件队列，取出 syncEv。
            - 调用 syncEv->callback()。callback 内部：唤醒 FFThread，然后自己阻塞（等待 FFThread 完成切换工作）。
        4. FFThread 醒来，重新获取锁，调用 EnterFastForward()。EnterFastForward 的子过程：
            1. `procTreeNode->enterFastForward()`：设置标志位 inFastForward = true。
            2. `PIN_RemoveInstrumentation()`：关键！ 再次清空 Pin 的代码缓存。
        5. `syncEv->signal();`，通知主线程继续，主模拟线程从 callback() 中返回，继续执行（此时已进入快进模式）。

### ThreadStart

当目标程序调用 pthread_create 或 clone 时，Pin 捕获到这一事件，并调用 ThreadStart。调用时机：目标线程刚刚被操作系统创建，但还没有开始执行任何具体的应用代码。

1. `if (procTreeNode->isInPause()) { ... }`：如果配置要求程序启动时先“暂停”（等待外部信号，比如调试器或多进程同步），这里会阻塞线程。利用 futex_lock 锁同一个变量两次。第一次初始化，第二次阻塞。直到外部工具解开这个锁，线程才会继续。
2. zsim 根据当前的模拟模式，决定给这个新线程分配什么样的“函数指针表” (fPtrs)：
    - 快进模式 (isInFastForward)：给线程装配“快进专用”的插桩函数（只计数，不模拟 Cache/Core）。
    - 影子/注册模式 (registerThreads)：给线程装配“空操作（NOP）”插桩。如果你只关心程序的某个特定区域（ROI）。线程虽然启动了，但在碰到 Magic Op 显式“注册”自己之前，zsim 当它不存在，不模拟它。
    - 正常详细模拟 (Normal Start)：调用 SimThreadStart(tid)

### SimThreadStart

真正的初始化逻辑：

1. `zinfo->sched->start(...)`：调用 Scheduler 类，在 gidMap 中创建一个新的 ThreadInfo 对象。此时线程状态被设为 STARTED，但还没有分配物理核心 (cid)。
2. `activeThreads[tid] = true;`：更新本地的活跃位图。
3. `fPtrs[tid] = joinPtrs;`：新线程不能直接冲进模拟循环，因为它错过了当前 Phase 的同步点。joinPtrs 是一组特殊的函数指针，它们会让线程在执行第一条指令时，强制调用 Scheduler::join()。这样，新线程会先去排队，领取一个 Core ID (cid)，等到下一个 Phase 开始时，才正式加入模拟大军。
4. `clearCid(tid);`：确保 cids[tid] 被初始化为无效值，等待 Scheduler 分配。

### ThreadFini

当目标线程执行完毕（pthread_exit 或从线程函数返回）时，Pin 调用此函数。调用时机：线程即将被销毁，此时它还可以执行一些清理代码。

1. `if (fPtrs[tid].type == FPTR_NOP) { return; }`：检查这个线程是不是一个“影子线程”（从未被激活过）。直接忽略，因为它从未在 Scheduler 里注册过，不需要注销。否则调用 SimThreadFini(tid);

### SimThreadFini

1. `zinfo->sched->finish(procIdx, tid);`：注销调度器。
2. `activeThreads[tid] = false;`、`cids[tid] = UNINITIALIZED_CID;`：更新状态，防止后续有野指针通过 tid 访问到错误的核心数据。

### Fini

当被模拟的目标程序（Guest Application）执行完毕，或者 zsim 决定强制结束模拟时，这一套流程会被触发。它们的任务是：确保多线程/多进程环境下的安全退出，并完整地保存统计数据 (zsim.out)。

Fini(int code, VOID * v)仅仅是 Pin 的回调入口。

### SimEnd

1. `if (__sync_bool_compare_and_swap(&perProcessEndFlag, 0, 1) == false)`: 一个进程可能有多个线程。当进程崩溃或调用 exit_group 时，Pin 可能会让多个线程同时触发 Fini / SimEnd。需要确保每个进程只有一个线程执行清理逻辑。
2. `bool lastToFinish = procTreeNode->notifyEnd();`：告诉全局共享内存（Shared Memory）这个进程已经结束了。原子递减 zinfo->globalActiveProcs（全局活跃进程数）。返回布尔值：如果减完之后变成了 0，说明我是全系统最后一个结束的进程。
3. Process 0 最终处理。在 zsim 的多进程模型中，进程 0 (Process 0) 被指定为“管家”。其他进程（Process 1, 2...）结束时只管自己死掉，但 Process 0 必须留到最后打扫战场。
    1. `while (zinfo->globalActiveProcs) usleep(100*1000);`：等待其他进程。
    2. `for (StatsBackend* backend : ...) backend->dump(false);`：dump统计数据。
    3. `zinfo->sched->notifyTermination()`：告诉调度器里的 Watchdog 线程，不用再查死锁了，一切要结束了。

### SyscallEnter

当目标程序执行 syscall 指令但尚未进入内核时，Pin 调用此函数。

1. 触发虚拟化补丁 (Patching)：检查此系统调用是否需要被拦截或修改。
2. 调度器通知 (Scheduling)：告诉调度器“我要离开用户态去内核办事了”，并交还 CPU (leave)。

子过程：

1. `bool isNopThread = ...;`、`bool isRetryThread = ...;`：检查当前线程是否是“影子线程”（不模拟）或者是正在“重试”某个被中断的系统调用。
2. `VirtSyscallEnter(tid, ctxt, std, ...);`：patch逻辑的核心入口
    - 代码中包含了针对 SYS_arch_prctl (CET protection) 和 SYS_clone3 的特殊处理。为了防止 Pin 崩溃或兼容 glibc，zsim 会手动跳过这些指令并返回错误码，假装内核不支持这些特性。
    - `postPatchFunctions[tid] = prePatchFunctions[syscall](...);`：查表分发，它使用系统调用号 (syscall) 作为索引，在 prePatchFunctions 表（我们在 VirtInit 中见过的）中查找对应的处理函数。
3. `if (fPtrs[tid].type != FPTR_JOIN && !zinfo->blockingSyscalls) { ... }`：如果这不是一个特殊的 Join 线程，且配置不允许阻塞式系统调用（默认情况）：
    - 交还 CPU (syscallLeave)：这告诉调度器：（线程 tid）正在离开用户态。我现在占用的物理核心 cid 必须被释放，以便其他线程使用。clearCid(tid)：清除本地的核心映射。
    - `fPtrs[tid] = joinPtrs;`：设置归来后的行为 (joinPtrs)，这意味着，当系统调用返回并执行下一条用户态指令时，该线程不会直接执行，而是会调用 joinPtrs 指向的函数，被迫去调度器那里排队重新申请核心 (join)。

### SyscallExit

当系统调用完成，内核即将返回用户态时，Pin 调用此函数。触发虚拟化后处理 (Post-Patching)：处理返回值，决定下一步行动。恢复状态：决定线程接下来是用 joinPtrs 排队，还是恢复正常执行，亦或是进入快进模式。

子过程：

1. `PostPatchAction ppa = VirtSyscallExit(tid, ctxt, std);`：它直接调用之前保存的 `postPatchFunctions[tid](...)`。例如，对于 SYS_futex，它的 Post-Patch 函数可能会检查返回值。如果 futex 决定阻塞线程，Post-Patch 函数会告诉调度器“这个线程睡着了”。返回值 ppa：这是一个动作指令（Action），告诉 SyscallExit 下一步该怎么做。
2. 下一步：
    - PPA_USE_JOIN_PTRS：意味着系统调用正常结束。设置 fPtrs[tid] = joinPtrs。线程回到用户态的第一件事就是去调度器排队 (join)。特例：如果 blockingSyscalls 为真（少见配置），则直接恢复 GetFuncPtrs()，不排队。
    - PPA_USE_RETRY_PTRS：意味着系统调用被信号中断（EINTR），需要重试。设置 fPtrs[tid] = retryPtrs。
3. `if (fPtrs[tid].type == FPTR_JOIN && procTreeNode->isInFastForward()) { ... }`：Fast-forwarding检查。如果线程在系统调用期间，模拟器切换到了快进模式 (Fast Forward)。那么线程就不应该再去 join（排队等待模拟核心）了，因为快进模式下不需要核心。
    - `SimThreadFini(tid)`：在 Scheduler 里注销（因为快进模式不需要 Scheduler 管理）。
    - `fPtrs[tid] = GetFFPtrs()`：直接切换到快进插桩函数。
4. 如果 `zinfo->terminationConditionMet` 为真（例如某个进程调用了 exit_group），则调用 SimEnd() 结束整个模拟。

### ContextChange

在正常的模拟流程中，程序按照“基本块 -> 系统调用 -> 基本块”的顺序执行。但是，信号（Signals）的存在打破了这种顺序。当操作系统向进程发送信号（比如 SIGSEGV 段错误，或者 SIGINT 中断）时，程序的控制流会突然从当前位置“瞬移”到信号处理函数，或者直接被终止。

ContextChange 就是 Intel Pin 提供的一个回调函数，用来通知 zsim 发生了非正常的上下文跳转（不是普通的函数调用或跳转）。

```c++
warn("[%d] ContextChange, reason %s, inSyscall %d", tid, reasonStr, inSyscall[tid]);
if (inSyscall[tid]) {
    SyscallExit(tid, to, SYSCALL_STANDARD_IA32E_LINUX, nullptr);
}
```

zsim 在 SyscallEnter 时会将 inSyscall[tid] 设为 true。正常情况下，SyscallExit 会将其设回 false。如果系统调用被信号打断（例如 EINTR），Pin 可能不会触发标准的 SyscallExit 回调，而是直接触发 ContextChange 跳转到信号处理函数。如果不处理，inSyscall[tid] 会一直保留为 true。等信号处理完回到主程序，下一次系统调用时，zsim 会断言失败（assert false）或者逻辑错乱。在这里强制手动调用 SyscallExit。这相当于帮 zsim “打补丁”，强行结束上一个系统调用的模拟状态，确保状态机复位。

```c++
if (reason == CONTEXT_CHANGE_REASON_FATALSIGNAL) {
    info("[%d] Fatal signal caught, finishing", tid);
    zinfo->sched->queueProcessCleanup(procIdx, getpid());
    SimEnd();
}
```

如果发生了段错误，queueProcessCleanup：通知调度器清理这个进程的遗体（释放核心、移除线程结构）。SimEnd()：这非常重要。即使程序崩溃了，zsim 也希望能够把已经收集到的统计数据（zsim.out）写盘保存，而不是直接随程序一起消失。这让用户能看到“崩溃前到底发生了什么”。

### PinCmd

它的核心任务是将用户在配置文件中简单的命令（例如 ls -l），转换成复杂的 Intel Pin 启动命令（例如 /opt/pin/pin -t /path/to/libzsim.so -config zsim.cfg ... -- ls -l），并处理极其繁琐的环境变量设置。

PinCmd 在两个截然不同的场景下被使用，调用链路略有不同：
1. Harness 启动时：在 zsim_harness 中，用于通过 fork + execvp 启动初始的模拟进程。
2. 模拟过程中：在 zsim.cpp (PinTool) 中，当被模拟程序调用 exec() 时，用于告诉 Pin 如何接管新的子进程。

### ContentionSim

ContentionSim 是 zsim 中负责模拟非核心组件（如缓存、内存、互连）时序竞争和延迟的核心引擎。它与基于 Pin 的功能模拟（Functional Simulation）和 CPU 核心的时序模拟（Timing Simulation）是并行且解耦的。

zsim 采用了一种Bound-Weave 模型：
- Bound Phase (Core Timing): 每个 CPU 核心独立运行，生成指令流和内存请求事件，估算核心内部延迟。这些事件被扔进 ContentionSim。
- Weave Phase (Contention Sim): ContentionSim 接管所有事件，模拟它们在缓存层次、网络和内存中的交互、排队和竞争，计算最终的系统延迟。

#### 初始化阶段

调用顺序： zsim.cpp:SimInit -> ContentionSim() -> postInit() -> initStats()

子过程与实现：
1. ContentionSim() 构造函数：
    - 创建域 (Domains)：zsim 为了并行化，将硬件系统划分为多个“域”（Domain）。通常一个 Domain 对应一个 Cache Bank 或 Memory Controller。domains 数组被分配。
    - 分配线程：创建 numSimThreads 个后台线程。每个线程负责模拟一部分 Domain。
    - 启动线程：PIN_SpawnInternalThread(SimThreadTrampoline, ...) 启动这些线程。它们启动后会立即在 futex_lock 上阻塞，等待唤醒。
    - 初始化队列：每个 Domain 都有一个 PrioQueue（优先队列），用于存放按时间排序的 TimingEvent。
2. postInit()：检查系统中有没有核心需要争用模拟 (needsCSim())。如果是纯功能模拟（如快进），skipContention 会被设为 true，跳过后续所有工作。

#### Bound Phase

在这个阶段，CPU 核心在 Pin 的控制下执行指令。ContentionSim 在此阶段主要扮演接收器的角色。动作：事件入队 (enqueue / enqueueSynced)

子过程：
1. 事件产生：当核心模拟到 Cache 访问、DRAM 刷新 (RefreshEvent) 或周期性任务 (TickEvent) 时，会创建一个继承自 TimingEvent 的对象。
2. 入队 (enqueueSynced)：
    - 核心调用此函数将事件放入目标 Domain 的优先队列中。
    - 必须加锁 (pqLock)，因为多个核心可能同时向同一个 L3 Cache Domain 发送请求。
    - 事件被标记上 startCycle（发起时间）。

特殊事件：跨域访问 (enqueueCrossing)
- 当一个事件从一个 Domain（比如 L2 Cache）流向另一个 Domain（比如 L3 Cache）时，会产生 CrossingEvent。
- enqueueCrossing 负责处理这种依赖关系，确保事件链的正确性。它可能会把新事件挂在旧事件（lastCrossing）后面，形成因果链。

#### Weave Phase

当所有核心都跑完当前的 Phase（比如 10000 周期）后，调度器调用 EndOfPhaseActions。

调用顺序： zsim.cpp:EndOfPhaseActions -> ContentionSim::simulatePhase(limit)

子过程与实现：
1. simulatePhase(limit)
    - 核心准备：调用所有 zinfo->cores[i]->cSimStart()。
    - 唤醒模拟线程：futex_unlock(&simThreads[i].wakeLock)。唤醒所有后台模拟线程。
    - 等待完成：主线程自己调用 futex_lock_nospin(&waitLock) 进入睡眠，等待所有后台线程干完活。
2. simThreadLoop -> simulatePhaseThread (后台线程工作逻辑)
    - 每个线程负责模拟它管辖的若干个 Domain（从 firstDomain 到 supDomain）。
    - 如果只管 1 个 Domain：这是一个优化路径。它只需简单地从 PrioQueue 中取出最早的事件 (dequeue)，调用事件的 run() 或 simulate() 方法，然后处理下一个，直到时间推进到 limit。
    - 如果管多个 Domain (多路归并)：
        - 它需要维护这些 Domain 之间的时间同步。
        - 它使用一个本地优先队列 (domPq) 来决定“哪个 Domain 的下一个事件最早发生”。
        - 复杂的同步逻辑：
            - 它不仅要处理内部事件，还要处理跨域事件 (CrossingEvent)。
            - CrossingEvent 有一个棘手的问题：源 Domain 的时间必须先推进，目标 Domain 才能安全地处理该事件。
            - 代码中的 stalledQueue 和 nextStalledQueue 就是为了处理这种依赖。如果一个 Domain 等待外部输入（prio > 0），它会被放入 Stall 队列暂时搁置。
3. 事件模拟 (TimingEvent::simulate / CrossingEvent::simulate)
    - 这是具体的硬件逻辑。例如 TickEvent 会调用 DDRMemory::tick()。
    - CrossingEvent 代表两个独立时钟域之间的握手。
        - simulate 会检查源 Domain 的时间 (srcDomainCycleAtDone)。
        - 如果源时间还没赶上来，CrossingEvent 会重新入队 (requeue)，稍后再试。这模拟了“数据还没传过来”的情况。
        - 一旦源时间满足条件，它计算传输延迟，更新目标时间。
4. 阶段结束
    - 当所有后台线程都把时间推进到了 limit，最后一个完成的线程会解锁 waitLock。
    - 主线程醒来，调用 cSimEnd() 通知核心，然后结束 EndOfPhaseActions。

### EventQueue

### Scheduler

### EndOfPhaseActions

### TakeBarrier

### AggregateStat

### PortVirtualizer

### NUMAMap

### ProcessStats

### ProcStats

### 