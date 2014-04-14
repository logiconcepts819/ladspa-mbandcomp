CC = gcc
ifeq ($(DEBUG),0)
DBGFLAGS = -O3
else ifeq ($(strip $(DEBUG)),)
DBGFLAGS = -O3
else
DBGFLAGS = -g3 -O0
endif
CFLAGS = --std=gnu99 -shared $(PLATFORM_CFLAGS) $(DBGFLAGS) -W -Wall
CD = cd
LDFLAGS = -lm
INSTALL = install
INSTALLFLAGS = -m0755
INSTALLROOT = /usr/local
INSTALLDIR = $(DESTDIR)$(INSTALLROOT)/lib/ladspa
MKDIR = mkdir
MKDIRFLAGS = -p
RM = rm
RMFLAGS = -Rf
RMDIR = rmdir
TRUE = true

TARGET = mbandcomp$(PLATFORM_EXT)
CFILES = mbandcomp.c linkwitzriley.c

.PHONY: all
all: $(TARGET)

$(TARGET): $(CFILES) $(RES_FILE)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: clean
clean:
	$(RM) $(RMFLAGS) $(RES_FILE) $(TARGET)

.PHONY: install
install:
	$(MKDIR) $(MKDIRFLAGS) $(INSTALLDIR) && \
		$(INSTALL) $(INSTALLFLAGS) $(TARGET) $(INSTALLDIR)

.PHONY: uninstall
uninstall:
	$(CD) $(INSTALLDIR) 2>/dev/null && { $(RM) $(RMFLAGS) $(TARGET) && \
		{ $(RMDIR) $(INSTALLDIR) 2>/dev/null || $(TRUE); }; } || \
		$(TRUE)
