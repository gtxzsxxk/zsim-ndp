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
4. 进程与多任务。使用PIN_AddFiniFunction函数

### Trace

### ThreadStart

### ThreadFini

### SyscallEnter

### SyscallExit

### ContextChange

### PinCmd

### ContentionSim

### EventQueue

### Scheduler

### AggregateStat

### PortVirtualizer

### NUMAMap

### ProcessStats

### ProcStats

### 