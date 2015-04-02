CXX ?= g++
CXXFLAGS = -g -ggdb 

OBJECTS = memory_table.o GPIO_over_DMA.o

%.o: %.cpp 
	$(CXX) $(CXXFLAGS) -fpermissive -c $< -o $@

all: $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -lpthread -o h

clean:
	rm -f *.o h
