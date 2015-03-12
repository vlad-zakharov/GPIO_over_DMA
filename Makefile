CXX ?= g++

OBJECTS = memory_table.o GPIO_over_DMA.o

%.o: %.cpp 
	$(CXX) -fpermissive -c $< -o $@

all: $(OBJECTS)
	$(CXX) $(OBJECTS) -lpthread -o h

clean:
	rm -f *.o h
