import os, sys
from os.path import join as joinpath

useIcc = False
#useIcc = True

def buildSim(cppFlags, dir, type, pgo=None):
    ''' Build the simulator with a specific base buid dir and config type'''

    buildDir = joinpath(dir, type)
    print("Building " + type + " zsim at " + buildDir)

    env = Environment(ENV = os.environ, tools = ['default', 'textfile'])
    env["CPPFLAGS"] = cppFlags

    allSrcs = [f for dir, subdirs, files in os.walk("src") for f in Glob(dir + "/*")]
    versionFile = joinpath(buildDir, "version.h")
    if os.path.exists(".git"):
        env.Command(versionFile, allSrcs + [".git/index", "SConstruct"],
            'printf "#define ZSIM_BUILDDATE \\"`date`\\"\\n#define ZSIM_BUILDVERSION \\"`python3 misc/gitver.py`\\"" >>' + versionFile)
    else:
        env.Command(versionFile, allSrcs + ["SConstruct"],
            'printf "#define ZSIM_BUILDDATE \\"`date`\\"\\n#define ZSIM_BUILDVERSION \\"no git repo\\"" >>' + versionFile)

    # Parallel builds?
    #env.SetOption('num_jobs', 32)

    # Use link-time optimization? It's still a bit buggy, so be careful
    #env['CXX'] = 'g++ -flto -flto-report -fuse-linker-plugin'
    #env['CC'] = 'gcc -flto'
    #env["LINKFLAGS"] = " -O3 -finline "
    if useIcc:
        env['CC'] = 'icc'
        env['CXX'] = 'icpc -ipo'

    ROOT = Dir('.').abspath

    # NOTE: These flags are for the 28/02/2011 2.9 PIN kit (rev39599). Older versions will not build.
    # NOTE (dsm 10 Jan 2013): Tested with Pin 2.10 thru 2.12 as well
    # NOTE: Original Pin flags included -fno-strict-aliasing, but zsim does not do type punning
    # NOTE (dsm 16 Apr 2015): Update flags code to support Pin 2.14 while retaining backwards compatibility
    # NOTE (gaomy May 2019): Set ABI version
    # NOTE (gaomy Sept 2020): Add -Wno-unused-function for the template ilog2
    env["CPPFLAGS"] += " -g -std=c++0x -Wall -Wno-unknown-pragmas -fomit-frame-pointer -fno-stack-protector"
    env["CPPFLAGS"] += " -MMD -DBIGARRAY_MULTIPLIER=1 -DUSING_XED -DTARGET_IA32E -DHOST_IA32E -fPIC -DTARGET_LINUX"
    env["CPPFLAGS"] += " -fabi-version=2"
    env["CPPFLAGS"] += " -Wno-unused-function"

    env["CPPPATH"] = []
    # Perform trace logging?
    ##env["CPPFLAGS"] += " -D_LOG_TRACE_=1"

    # Uncomment to get logging messages to stderr
    ##env["CPPFLAGS"] += " -DDEBUG=1"

    # Be a Warning Nazi? (recommended)
    # env["CPPFLAGS"] += " -Werror "

    # Non-pintool libraries
    env["LIBPATH"] = []
    env["LIBS"] = ["config", "hdf5", "hdf5_hl"]

    env["LINKFLAGS"] = ""

    if useIcc:
        # icc libs
        env["LINKFLAGS"] += " -Wl,-R/data/sanchez/tools/intel/composer_xe_2013.1.117/compiler/lib/intel64/"

    # Use non-standard library paths if defined
    if "LIBCONFIGPATH" in os.environ:
        LIBCONFIGPATH = os.environ["LIBCONFIGPATH"]
        env["LINKFLAGS"] += " -Wl,-R" + joinpath(LIBCONFIGPATH, "lib")
        env["LIBPATH"] += [joinpath(LIBCONFIGPATH, "lib")]
        env["CPPPATH"] += [joinpath(LIBCONFIGPATH, "include")]

    if "HDF5PATH" in os.environ:
        HDF5PATH = os.environ["HDF5PATH"]
        env["LINKFLAGS"] += " -Wl,-R" + joinpath(HDF5PATH, "lib")
        env["LIBPATH"] += [joinpath(HDF5PATH, "lib")]
        env["CPPPATH"] += [joinpath(HDF5PATH, "include")]

    if "MBEDTLSPATH" in os.environ:
        MBEDTLSPATH = os.environ["MBEDTLSPATH"]
        env["LINKFLAGS"] += " -Wl,-R" + joinpath(MBEDTLSPATH, "lib")
        env["CPPPATH"] += [joinpath(MBEDTLSPATH, "include")]
        env["CPPFLAGS"] += " -D_WITH_MBEDTLS_=1 "

    if "POLARSSLPATH" in os.environ:
        POLARSSLPATH = os.environ["POLARSSLPATH"]
        env["LINKFLAGS"] += " -Wl,-R" + joinpath(POLARSSLPATH, "lib")
        env["CPPPATH"] += [joinpath(POLARSSLPATH, "include")]
        env["CPPFLAGS"] += " -D_WITH_POLARSSL_=1 "

    # Only include DRAMSim if available
    if "DRAMSIMPATH" in os.environ:
        DRAMSIMPATH = os.environ["DRAMSIMPATH"]
        env["LINKFLAGS"] += " -Wl,-R" + DRAMSIMPATH
        env["CPPPATH"] += [DRAMSIMPATH]
        env["CPPFLAGS"] += " -D_WITH_DRAMSIM_=1 "

    # addr2line from GNU binutils is used as an independent third-party executable called by zsim when backtracing bugs.
    # Some versions of addr2line seems not working with Pin and causes segfault (e.g., addr2line 2.26.1 + pin 3.11).
    # Try build a different version manually and specify the path to the executable here.
    if "ADDR2LINEBIN" in os.environ:
        ADDR2LINEBIN = os.environ["ADDR2LINEBIN"]
        env["CPPFLAGS"] += ' -DADDR2LINEBIN="' + ADDR2LINEBIN + '" '

    env["CPPPATH"] += ["."]

    # Harness needs these defined
    env["CPPFLAGS"] += ' -DZSIM_PATH="' + joinpath(ROOT, joinpath(buildDir, "libzsim.so")) + '" '

    env.SConscript("src/SConscript", variant_dir=buildDir, exports= {'env' : env.Clone()})

####

AddOption('--buildDir', dest='buildDir', type='string', default="build/", nargs=1, action='store', metavar='DIR', help='Base build directory')
AddOption('--d', dest='debugBuild', default=False, action='store_true', help='Do a debug build')
AddOption('--o', dest='optBuild', default=False, action='store_true', help='Do an opt build (optimized, with assertions and symbols)')
AddOption('--r', dest='releaseBuild', default=False, action='store_true', help='Do a release build (optimized, no assertions, no symbols)')
AddOption('--p', dest='pgoBuild', default=False, action='store_true', help='Enable PGO')
AddOption('--pgoPhase', dest='pgoPhase', default="none", action='store', help='PGO phase (just run with --p to do them all)')


baseBuildDir = GetOption('buildDir')
buildTypes = []
if GetOption('debugBuild'): buildTypes.append("debug")
if GetOption('releaseBuild'): buildTypes.append("release")
if GetOption('optBuild') or len(buildTypes) == 0: buildTypes.append("opt")

march = "core2" # ensure compatibility across condor nodes
#march = "native" # for profiling runs

buildFlags = {"debug": "-g -O0",
              "opt": "-march=%s -g -O3 -funroll-loops" % march, # unroll loops tends to help in zsim, but in general it can cause slowdown
              "release": "-march=%s -O3 -DNASSERT -funroll-loops -fweb" % march} # fweb saves ~4% exec time, but makes debugging a world of pain, so careful

pgoPhase = GetOption('pgoPhase')

# The PGO flow calls scons recursively. Hacky, but pretty much the only option:
# scons can't build the same file twice, and although gcc enables you to change
# the fprofile path, it considers the whole relative path as the filename
# (e.g., build/opt/zsim.os), and all hell breaks loose when it tries to create
# files in another dir. And because it uses checksums for filenames, it breaks
# when you move the files. Check the repo for a version that tries this.
if GetOption('pgoBuild'):
    for type in buildTypes:
        print("Building PGO binary")
        root = Dir('.').abspath
        testsDir = joinpath(root, "tests")
        trainCfgs = [f for f in os.listdir(testsDir) if f.startswith("pgo")]
        print("Using training configs", trainCfgs)

        baseDir = joinpath(baseBuildDir, "pgo-" + type)
        genCmd = "scons -j16 --pgoPhase=generate-" + type
        runCmds = []
        for cfg in trainCfgs:
            runCmd = "mkdir -p pgo-tmp && cd pgo-tmp && ../" + baseDir + "/zsim ../tests/" + cfg + " && cd .."
            runCmds.append(runCmd)
        useCmd = "scons -j16 --pgoPhase=use-" + type
        Environment(ENV = os.environ).Command("dummyTgt-" + type, [], " && ".join([genCmd] + runCmds + [useCmd]))
elif pgoPhase.startswith("generate"):
    type = pgoPhase.split("-")[1]
    buildSim(buildFlags[type], baseBuildDir, "pgo-" + type, "generate")
elif pgoPhase.startswith("use"):
    type = pgoPhase.split("-")[1]
    buildSim(buildFlags[type], baseBuildDir, "pgo-" + type, "use")
    baseDir = joinpath(baseBuildDir, "pgo-" + type)
    Depends(Glob(joinpath(baseDir, "*.os")), "pgo-tmp/zsim.out") #force a rebuild
else:
    for type in buildTypes:
        buildSim(buildFlags[type], baseBuildDir, type)
