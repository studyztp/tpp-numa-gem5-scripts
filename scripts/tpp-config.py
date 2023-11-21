# Copyright (c) 2021 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse
import m5
from gem5.utils.requires import requires
from gem5.utils.override import overrides
from gem5.components.boards.arm_board import ArmBoard
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.memory import SingleChannelDDR4_2400
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.components.processors.cpu_types import CPUTypes
from gem5.isas import ISA
from gem5.simulate.simulator import Simulator
from gem5.simulate.simulator import ExitEvent
from gem5.resources.resource import Resource, AbstractResource, DiskImageResource, KernelResource,obtain_resource
from pathlib import Path
from utils.NumaArmBoard import ArmDMBoard
from gem5.components.memory.dram_interfaces.ddr4 import DDR4_2400_8x8
from utils.RemoteMemory import RemoteChanneledMemory

parser = argparse.ArgumentParser()
parser.add_argument(
    "--bench",
    type=str,
    required=True,
    help="benchmark",
)

args = parser.parse_args()

from utils.DMCache import ClassicPL1PL2DMCache

# Here we setup the parameters of the l1 and l2 caches.
cache_hierarchy = ClassicPL1PL2DMCache(
    l1d_size="32kB",
    l1i_size="32kB",
    l2_size="256kB"
)

# Memory: Dual Channel DDR4 2400 DRAM device.

local_memory = DualChannelDDR4_2400(size="256MB")
remote_memory = RemoteChanneledMemory(
        DDR4_2400_8x8,
        2,
        64,
        size="8GB",
        remote_offset_latency=750
)

# Here we setup the processor. We use a simple processor.
processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING, isa=ISA.ARM, num_cores=2
)

board = ArmDMBoard(
    clk_freq="3GHz",
    processor=processor,
    local_memory=local_memory,
    remote_memory=remote_memory,
    cache_hierarchy=cache_hierarchy,
)

cmd = [
    "echo running mount -t sysfs - /sys;",
    "mount -t sysfs - /sys;",
    "echo running mount -t proc - /proc;",
    "mount -t proc - /proc;",
    "echo running numastat;",
    "numastat;",
    "echo cat numa_balancing;",
    "cat /proc/sys/kernel/numa_balancing;",
    "echo write vmstat to host;",
    "cat /proc/vmstat > init_vmstat.txt;",
    "m5 writefile init_vmstat.txt;",
    f"echo run {args.bench}.A.x;",
    "m5 workbegin;",
    f"/home/ubuntu/NPB/NPB3.4-OMP/bin/{args.bench}.A.x;",
    "cat /proc/vmstat > end_vmstat.txt;",
    "m5 writefile end_vmstat.txt;",
    "sleep 1;",
    "m5 exit;"
]

board.set_kernel_disk_workload(
    kernel=KernelResource("/scr/studyztp/experiments/dm/disk/kg-5-18-vmlinux"),
    # kernel=obtain_resource(resource_id="arm64-linux-kernel-5.15.36"),
    bootloader=obtain_resource(resource_id="arm64-bootloader-foundation"),
    disk_image=DiskImageResource(
        "/scr/studyztp/experiments/dm/disk/arm64-hpc-2204.img",
        root_partition="1"),
    readfile_contents="".join(cmd),
    checkpoint=Path("/scr/studyztp/experiments/dm/bootup-cpt")
)

def take_cpt():
    print("reach m5 checkpoint\n")
    m5.checkpoint(args.cpt_path)
    yield False

def workbegin_handler():
    print("reach first workbegin\nreset stats\n")
    m5.stats.reset()
    yield False
    print("reach second workbegin\ndump and reset stats\n")
    m5.stats.dump()
    m5.stats.reset()
    yield False

def workend_handler():
    print("reached workend\ndump and reset stats\n")
    m5.stats.dump()
    m5.stats.reset()
    yield False
    

simulator = Simulator(
    board=board,
    on_exit_event = {
        ExitEvent.WORKBEGIN : workbegin_handler(),
        ExitEvent.WORKEND : workend_handler(),
        ExitEvent.CHECKPOINT : take_cpt()
    }
)

simulator.run()

print("Reach the end of script\n")
