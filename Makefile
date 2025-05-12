# ─── Makefile ────────────────────────────────────────────────────────────────

# 编译器和工具
CXX      ?= g++
AR       ?= ar
RM       ?= rm -f

# 项目相关
PROJ     := dk_line_op
LIB      := lib$(PROJ).a

SRCDIR   := src
INCDIR   := include
THIRDP   := thirdparty/eigen

# 源文件与目标文件
SRCS     := $(wildcard $(SRCDIR)/*.cpp)
OBJS     := $(SRCS:.cpp=.o)

# 编译选项
CXXFLAGS := -std=c++17 -O3 -Wall -I$(INCDIR) -I$(THIRDP)

# 安装前缀，可通过 'make install PREFIX=/your/path' 覆盖
PREFIX    ?= ../DK_LineOp
LIBDIR     = $(PREFIX)/lib
INCDST     = $(PREFIX)/include

.PHONY: all clean install uninstall

# 默认目标：生成静态库
all: $(LIB)

# 打包静态库
$(LIB): $(OBJS)
	$(AR) rcs $@ $^

# 编译 .cpp -> .o
$(SRCDIR)/%.o: $(SRCDIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 删除编译产物
clean:
	rm -rf build

# # 安装到 $(PREFIX)
# install: $(LIB)
# 	mkdir -p $(LIBDIR) $(INCDST)
# 	cp $(LIB)        $(LIBDIR)/
# 	cp $(INCDIR)/*.h $(INCDST)/

build:
	mkdir -p build && cd build && \
	cmake .. && \
	cmake --build . -- 

# 卸载
uninstall:
	$(RM) $(LIBDIR)/$(LIB)
	$(RM) $(addprefix $(INCDST)/, $(notdir $(wildcard $(INCDIR)/*.h)))

# ──────────────────────────────────────────────────────────────────────────────
