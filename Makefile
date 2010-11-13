all: demo 

demo: main.cpp movement.h physics.h skybox.o
	g++ main.cpp skybox.o -o demo -losg -losgDB -losgViewer -losgCal -losgUtil -losgViewer -lBulletDynamics -lBulletCollision -lLinearMath -I/usr/include/bullet -Wall -Werror

skybox.o: skybox.cpp skybox.h
	g++ -c skybox.cpp -o skybox.o

clean:
	rm demo *.o
