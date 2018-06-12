all:
	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/linaro/serial/build/devel/lib; sudo g++ -ggdb thread2.cpp -lpthread -o video `pkg-config --cflags --libs opencv` -L /home/linaro/serial/build/devel/lib/ -lserial
