ladspa-mbandcomp
================

A LADSPA plugin for multiband compression, which uses Linkwitz-Riley filters for the frequency splits.  This is an extension of my compressor plugin implementation in VLC.

#Building ladspa-mbandcomp

##MinGW in Windows

Execute the following in a MinGW shell:

```
make -f Makefile.win
```
If you wish to build a 64-bit plugin, use the 64-bit MinGW shell.  If you wish
to build a 32-bit plugin, use the 32-bit MinGW shell.

##MinGW in Linux

If you wish to build a 64-bit plugin, execute the following in a terminal:
```
./win64make
```
If you wish to build a 32-bit plugin, execute the following:
```
./win32make
```
If gcc and windres go by different filenames than what are used in the
win32make and win64make scripts, you can override the CC and WINDRES variables
with the correct filenames.  For example, for the old version of MinGW, one
would run
```
CC=i586-mingw32msvc-gcc WINDRES=i586-mingw32msvc-windres ./win32make
```

##Linux

Execute the following in a terminal:
```
make -f Makefile.unix
```

#Installation

##Windows

Set the LADSPA_PATH environment variable with all possible paths you wish to
designate for LADSPA plugins. For example, on Windows 64-bit systems, one may
want to designate a 32-bit directory and a 64-bit directory for LADSPA plugins,
so the value of LADSPA_PATH would be something like
```
C:\Program Files\LADSPA;C:\Program Files (x86)\LADSPA
```
After setting the environment variable, copy the compiled DLL to the
appropriate location(s).

##Linux

Copy the compiled .so to a location that is recognized by LADSPA hosts such as
/usr/lib/ladspa or /usr/local/lib/ladspa.
