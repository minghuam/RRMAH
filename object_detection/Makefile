CC = g++
CFLAGS = -std=c++0x
INCLUDES = -I./ -I../util
LFLAGS = $(shell pkg-config opencv --libs)$

all: mask_sampler test

mask_sampler: mask_sampler.cpp mask_generator.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -o mask_sampler mask_sampler.cpp mask_generator.cpp $(LFLAGS) 

test: test.cpp obj_color_feature.cpp obj_global_feature.cpp obj_detector.cpp obj_tracker.cpp
	$(CC) $(CFLAGS) $(INCLUDES) -o test test.cpp obj_color_feature.cpp obj_global_feature.cpp obj_detector.cpp obj_tracker.cpp $(LFLAGS)

clean:
	rm -f *.o *.~ mask_sampler test