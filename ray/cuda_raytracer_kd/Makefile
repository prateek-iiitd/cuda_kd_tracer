
objects = main.o triangle.o camera.o aabb.o

all: $(objects)
			nvcc -arch=sm_20 --ptxas-options=-v -G -lm -lIL -lILU -L../devil/lib -lstdc++ $(objects) -o app

%.o: %.cpp
			nvcc -x cu -arch=sm_20 --ptxas-options=-v -G -I. -I../common/inc -I../devil/include -dc $< -o $@

clean:
			rm -f *.o app
