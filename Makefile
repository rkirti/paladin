CC = g++
CFLAGS = -I/usr/include/bullet -pg
WFLAGS = -Wall 
LDFLAGS = -losg -losgDB -losgViewer -losgCal -losgUtil -losgViewer -lBulletDynamics -lBulletCollision -lLinearMath  -lOpenThreads


demo: main.cpp hud.cpp osgdraw.cpp physics.cpp skybox.cpp
	g++ $^ -o $@  $(LDFLAGS) $(CFLAGS) $(WFLAGS)

%.o: %.cpp
	gcc -c $*.cpp $(CFLAGS)  $(WFLAGS)

clean:
	rm demo *.o
