This is a C project for ARM Cortex M0+, M0, M3 and M4
[LPC](http://www.nxp.com/products/microcontrollers/) microcontrollers. The
[GNU Toolchain](http://en.wikipedia.org/wiki/GNU_toolchain) is used for
compilation, but the
[LPCXpresso IDE](http://www.nxp.com/techzones/microcontrollers-techzone/tools-ecosystem/lpcxpresso.html)
is still required for downloading and debugging via NXP's LPC-Link.

**The master branch is not intended to be built - Rather it is just a
  base that implementations for particular chips can be built upon.**

To switch to a branch that will do something useful:
* On GitHub: Select a branch from the drop-down menu above.
* On the command line: `git branch` to see a list of branches and
  switch to one with `git checkout <branch>`.

## Prerequisites ##

[GNU Make](http://www.gnu.org/software/make/) and the following standard
utilities are required: `cat`, `echo`, `find`, `grep`, `mkdir`, `rm`, `sed` and
`shuf`. If you're running any sensible desktop linux then these will already be
installed.

You will need to aquire
[GNU Tools for ARM Embedded Processors](https://launchpad.net/gcc-arm-embedded/).

##### On Ubuntu

```
sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update && sudo apt-get install gcc-arm-none-eabi
```

##### Otherwise

[Download](https://launchpad.net/gcc-arm-embedded/+download) and add
the `bin/` directory to your path. If you use
[Bash](https://en.wikipedia.org/wiki/Bash_(Unix_shell\)), you might be
able to do this by adding `export
PATH=$PATH:/path/to/gnu/tools/for/arm/bin/` to your
`~/.bashrc`.

##### Not Recommended

You could build your own tools from source. Have a look at
[summon-arm-toolchain](http://summon-arm-toolchain.org/).

## Configuration ##

Most of the configuration options related to the chip are located in
[`makefile.conf`](makefile.conf). Please read the file itself for full details on each of the
options. You should probably run `make clean` after editing [`makefile.conf`](makefile.conf).

The sizes and locations of your chip's memory locations should be defined in
[`mem.ld`](chip/mem.ld). Any other changes to the layout of the final binary
(such as using external/AHB RAM or placing functions in RAM) should be made to
the linker script [`sections.ld`](sections.ld)

If you wish to download / debug using NXP's LPC-Link, you will need to edit the
`LPCINSTALL` variable in `Makefile` to point to the `bin/` directory of your
[LPCXpresso IDE](http://www.nxp.com/techzones/microcontrollers-techzone/tools-ecosystem/lpcxpresso.html)
install.

## Usage ##

After making sure [sources.mk](#sources) is up-to-date, simply run `make` to compile. 

If you wish to download / debug using NXP's LPC-Link, run `make lpc-link` to
flash the LPC-Link's own firmware and start a server which can be connected to
by [GDB](http://www.gnu.org/software/gdb/). This will not return - you will need
to open a new terminal to continue. You can only do this once before you will
need to unplug and reconnect (or do some USB trickery to) the LPC-Link.

The makefile picks a random 5-digit port to start the debug server, and modifies
the `connect` command defined in [`.gdbscript`](.gdbscript) to point to it. Therefore you
should run GDB (or more precisely `arm-none-eabi-gdb`) with the
`--connect=.gdbscript` flag.

## sources ##

`make sources` is responsible for building [`sources.mk`](`sources.mk`), which is a list of all
the source files in the project (except those listed in the `OTHER_SOURCES`
section of [`makefile.conf`](makefile.conf)). When run `make sources` will add every `.c` and
`.S` file in the sources directory (currently defined as `src/`) to [`sources.mk`](sources.mk). However this list should be checked by hand to be certain that only the
files you intended are being linked into the build.

## Emacs ##

A Directory Local Variables File [`.dir-locals.el`](.dir-locals.el) exists in the root of the
project, and this has the following effects on emacs:

### Fixed default-directory ###

The default directory is fixed as the root of the project wherever you are
within the project. As the makefile needs to be run from the root of the
project, this means that `M+x compile` will always start a compile whichever
file you are editing.

### Custom GDB ###

`M+x gdb` is set to use `arm-none-eabi-gdb` rather than the default GDB for your
machine. GDB is also instructed to read in [`.gdbscript`](.gdbscript) as a command file.

## Other notes ##

Wherever possible use
[Function Attributes](http://gcc.gnu.org/onlinedocs/gcc/Function-Attributes.html)
and
[Variable Attrubutes](http://gcc.gnu.org/onlinedocs/gcc/Variable-Attributes.html)
rather than editing [`sections.ld`](sections.ld).

## Sources & Licensing ##

See [LICENSE.md](LICENSE-lpc-toolchain.md)
