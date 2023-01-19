all: build run

build: 
	gcc main.c -o bin2 -lwiringPi -lpthread
run:    
	./bin2 "/dev/i2c-1"
clean:
	rm -rf bin