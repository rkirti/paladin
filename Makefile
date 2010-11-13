demo: main.cpp movement.h physics.h
	g++ main.cpp -o demo -losg -losgDB -losgViewer -losgCal -losgUtil -losgViewer -lBulletDynamics -lBulletCollision -lLinearMath -I/usr/include/bullet -Wall -Werror
clean:
	rm demo
