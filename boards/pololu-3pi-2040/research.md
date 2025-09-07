# Research

## RP2040

In-depth notes on the rp2040 microcontroller.

### Memory

need to understand how these fabrics work
seperate sram banks allow for dual core access on the same cycle
there are registers for counting accesses

- important for our performance analysis
- debugging

- ahb lite
  - used for connecting to the rams and flash
  - masters are the cores and dma
  - made up of splitters and arbiters
  - pipelined?
- apb bridge
  - splitter connects to system control registers

the abstraction is memory mapped registers
the bus fabric routes to the correct on each cycle

all masters can access four different
ports at the same time

- there is bus contention, would waste a cycle
- this should be simulated

- TODO: need to see how verilog implements this
  - how to implement in chisel
- TODO: need to understand which slaves have zero wait states
- TODO: investigate other performance probe hooks

### Cores

two cortex-m0+

### PIO

The peripheral has 9 instructions.
Use pioasm to assemble programs.

The basic 


## Pololu 3pi+

## Agent

The agent seems to loop over changes that were done and undo them.
This is a waste of how the agent operates

Also, for embedded development simple operations are ofen the best
Need a system prompt and a set of tools that do very basic operations
Optimize for minimum lines of code changed

```
Make as few changes as possible.
Explain to me the 
```

Need to tune the system prompts for embedded development.
Usually this means the specs are minimal implementations.

It would be useful for the system to know if the program
- has halted, some sort of watch dog
- see prints and terminate the process that is outputing them

adding constraints in the spec helps steer the agent
this is really reinforcement learning 101

TODO: review datasheets
- Have an agent integration that ingests and parses datasheets

TODO: have an integration that can look at registers
- Help me understand the operation of the registers using a probe-rs 


```
The same error is occuring, all values are reporting 1024 when
printed. This is due to the read timeout. Do not modify the qtr_ex.c and qtr_ex.pio. Use them as
references for the c implementation of the behavior. Modify the
rust implementation to mimic the behavior. Find a simple a
solution as possible with the fewest number of changes. Explain to
me what the c implementation.
```

if the agent is of task, restarting and using a condensed prompt
is the best method




