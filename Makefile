
CC = g++
CFLAGS = -I/usr/include/bullet
WFLAGS = -Wall 
LDFLAGS = -losg -losgDB -losgViewer -losgCal -losgUtil -losgViewer -lBulletDynamics -lBulletCollision -lLinearMath


demo: main.cpp movement.h physics.o skybox.o hud.o osgdraw.o
	g++ $^ -o $@  $(LDFLAGS) $(CFLAGS) $(WFLAGS)

%.o: %.cpp
	gcc -c $*.cpp $(CFLAGS)  $(WFLAGS)

clean:
	rm demo *.o
