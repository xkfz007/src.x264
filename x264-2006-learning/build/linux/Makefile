# Makefile

TARGET:=x264
SRCDIR:=../../src
OBJDIR:=obj
$(shell if !([ -d $(OBJDIR) ]);then mkdir $(OBJDIR);fi)

CC:=gcc

FLAGS:=-Wall -W
LIBS :=-lm

#DBG=0
ifdef DBG
    FLAGS+=-g
else
    FLAGS+=-O2
endif

SRC:=$(wildcard $(SRCDIR)/*.c)
OBJ:=$(patsubst %.c,$(OBJDIR)/%.o,$(notdir $(SRC)))

$(TARGET):$(OBJ)
	$(CC) -o $@ $^ $(FLAGS) $(LIBS)
$(OBJDIR)/%.o:$(SRCDIR)/%.c 
	$(CC) -c $< -o $@ $(FLAGS)
clean:
	rm -f $(OBJDIR)/*.o $(TARGET)
