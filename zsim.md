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