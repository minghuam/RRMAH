CC = g++
CFLAGS = -std=c++0x
INCLUDES = -I.
LFLAGS = $(shell pkg-config opencv --libs)$

all: mask_sampler

mask_sampler: mask_sampler.cpp mask_generator.cpp
	$(CC) $(CFLAGS) -o mask_sampler mask_sampler.cpp mask_generator.cpp $(LFLAGS) 

clean:
	rm -f *.o *.~ mask_sampler