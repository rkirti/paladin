all: demo 

demo: main.cpp movement.h physics.h skybox.o hud.o
	g++ main.cpp skybox.o hud.o -o demo -losg -losgDB -losgViewer -losgCal -losgUtil -losgViewer -lBulletDynamics -lBulletCollision -lLinearMath -I/usr/include/bullet -Wall -Werror

skybox.o: skybox.cpp skybox.h
	g++ -c skybox.cpp -o skybox.o

hud.o: hud.cpp hud.h
	g++ -c hud.cpp -o hud.o

clean:
	rm demo *.o
