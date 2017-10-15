CC = g++
CFLAGS = -o -Wall

mot_ctrl: mot_ctrl_test.cpp
	$(CC) $(CFLAGS) -o $@ $< `sdl-config --cflags --libs` -lm

clean:
	rm -f mot_ctrl
