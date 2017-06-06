COMPILER = g++
FLAGS    = -std=c++14 -O3 -Wall -Werror -Wextra
MSGPACK  = $(shell pkg-config --libs --cflags msgpack)
ZEROMQ   = $(shell pkg-config --libs --cflags libzmq)
SO_DEPS  = -lpthread -lAria -ldl -lm -lrt -larmadillo $(MSGPACK) $(ZEROMQ) -I../include

TARGET = server

all: $(TARGET)

clean:
	rm -rf $(TARGET)

$(TARGET):$(TARGET).cpp
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS)